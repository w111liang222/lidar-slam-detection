import torch
import torch.nn as nn
import torch.nn.functional as F

wid_mul = 0.75
base_channels = int(wid_mul * 32)
class Mish(torch.nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, x):
        x = x * (torch.tanh(torch.nn.functional.softplus(x)))
        return x

class Upsample(nn.Module):
    def __init__(self, in_channels, out_channels, scale_factor=2):
        super(Upsample, self).__init__()

        self.upsample = nn.Sequential(
            nn.ConvTranspose2d(
                            in_channels, out_channels,
                            scale_factor,
                            stride=scale_factor, bias=False
            ),
            nn.BatchNorm2d(out_channels),
            nn.ReLU()
        )

    def forward(self, x,):
        x = self.upsample(x)
        return x

class Conv_Bn_Activation(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride, activation, bn=True, bias=False):
        super().__init__()
        pad = (kernel_size - 1) // 2

        self.conv = nn.ModuleList()
        if bias:
            self.conv.append(nn.Conv2d(in_channels, out_channels, kernel_size, stride, pad))
        else:
            self.conv.append(nn.Conv2d(in_channels, out_channels, kernel_size, stride, pad, bias=False))
        if bn:
            self.conv.append(nn.BatchNorm2d(out_channels))
        if activation == "mish":
            self.conv.append(Mish())
        elif activation == "relu":
            self.conv.append(nn.ReLU(inplace=True))
        elif activation == "leaky":
            self.conv.append(nn.LeakyReLU(0.1, inplace=True))
        elif activation == "linear":
            pass
        else:
            print("activate error !!! {} {} {}".format(sys._getframe().f_code.co_filename,
                                                       sys._getframe().f_code.co_name, sys._getframe().f_lineno))

    def forward(self, x):
        for l in self.conv:
            x = l(x)
        return x

class ResBlock(nn.Module):
    """
    Sequential residual blocks each of which consists of \
    two convolution layers.
    Args:
        ch (int): number of input and output channels.
        nblocks (int): number of residual blocks.
        shortcut (bool): if True, residual tensor addition is enabled.
    """

    def __init__(self, ch, nblocks=1, shortcut=True):
        super().__init__()
        self.shortcut = shortcut
        self.module_list = nn.ModuleList()
        for i in range(nblocks):
            resblock_one = nn.ModuleList()
            resblock_one.append(Conv_Bn_Activation(ch, ch, 1, 1, 'relu'))
            resblock_one.append(Conv_Bn_Activation(ch, ch, 3, 1, 'relu'))
            self.module_list.append(resblock_one)

    def forward(self, x):
        for module in self.module_list:
            h = x
            for res in module:
                h = res(h)
            x = x + h if self.shortcut else h
        return x

class DownSample1(nn.Module):
    def __init__(self, input_channels):
        super().__init__()
        self.conv1 = Conv_Bn_Activation(input_channels, base_channels, 3, 1, 'relu')

        self.conv2 = Conv_Bn_Activation(base_channels, base_channels * 2, 3, 2, 'relu')
        self.conv3 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')
        # [route]
        # layers = -2
        self.conv4 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')

        self.conv5 = Conv_Bn_Activation(base_channels * 2, base_channels, 1, 1, 'relu')
        self.conv6 = Conv_Bn_Activation(base_channels, base_channels * 2, 3, 1, 'relu')
        # [shortcut]
        # from=-3
        # activation = linear

        self.conv7 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')
        # [route]
        # layers = -1, -7
        self.conv8 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')

    def forward(self, input):
        x1 = self.conv1(input)
        x2 = self.conv2(x1)
        x3 = self.conv3(x2)
        # route -2
        x4 = self.conv4(x2)
        x5 = self.conv5(x4)
        x6 = self.conv6(x5)
        # shortcut -3
        x6 = x6 + x4

        x7 = self.conv7(x6)
        # [route]
        # layers = -1, -7
        x7 = torch.cat([x7, x3], dim=1)
        x8 = self.conv8(x7)
        return x8


class DownSample2(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 2, 'relu')
        self.conv2 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        # r -2
        self.conv3 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')

        self.resblock = ResBlock(ch=base_channels * 2, nblocks=3)

        # s -3
        self.conv4 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')
        # r -1 -10
        self.conv5 = Conv_Bn_Activation(base_channels * 4, base_channels * 4, 1, 1, 'relu')

    def forward(self, input):
        x1 = self.conv1(input)
        x2 = self.conv2(x1)
        x3 = self.conv3(x1)

        r = self.resblock(x3)
        x4 = self.conv4(r)

        x4 = torch.cat([x4, x2], dim=1)
        x5 = self.conv5(x4)
        return x5


class DownSample3(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 2, 'relu')
        self.conv2 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv3 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')

        self.resblock = ResBlock(ch=base_channels * 4, nblocks=4)
        self.conv4 = Conv_Bn_Activation(base_channels * 4, base_channels * 4, 1, 1, 'relu')
        self.conv5 = Conv_Bn_Activation(base_channels * 8, base_channels * 8, 1, 1, 'relu')

    def forward(self, input):
        x1 = self.conv1(input)
        x2 = self.conv2(x1)
        x3 = self.conv3(x1)

        r = self.resblock(x3)
        x4 = self.conv4(r)

        x4 = torch.cat([x4, x2], dim=1)
        x5 = self.conv5(x4)
        return x5

class DownSample4(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = Conv_Bn_Activation(base_channels * 8, base_channels * 16, 3, 2, 'relu')
        self.conv2 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')
        self.conv3 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')

        self.resblock = ResBlock(ch=base_channels * 8, nblocks=6)
        self.conv4 = Conv_Bn_Activation(base_channels * 8, base_channels * 8, 1, 1, 'relu')
        self.conv5 = Conv_Bn_Activation(base_channels * 16, base_channels * 16, 1, 1, 'relu')

    def forward(self, input):
        x1 = self.conv1(input)
        x2 = self.conv2(x1)
        x3 = self.conv3(x1)

        r = self.resblock(x3)
        x4 = self.conv4(r)

        x4 = torch.cat([x4, x2], dim=1)
        x5 = self.conv5(x4)
        return x5

class Neck(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv0 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')
        self.conv1 = Conv_Bn_Activation(base_channels * 32, base_channels * 16, 1, 1, 'relu')
        self.conv1_1=Conv_Bn_Activation(base_channels * 16, base_channels * 16, 3, 1, 'relu')
        self.conv2 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')
        self.upsample0 = Upsample(base_channels * 8, base_channels * 8)

        self.conv3 = Conv_Bn_Activation(base_channels * 8, base_channels * 8, 1, 1, 'relu')
        # SPP
        self.maxpool1 = nn.MaxPool2d(kernel_size=5, stride=1, padding=5 // 2)
        self.maxpool2 = nn.MaxPool2d(kernel_size=9, stride=1, padding=9 // 2)
        self.maxpool3 = nn.MaxPool2d(kernel_size=13, stride=1, padding=13 // 2)

        # R -1 -3 -5 -6
        # SPP
        self.conv4 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')
        self.conv5 = Conv_Bn_Activation(base_channels * 8, base_channels * 16, 3, 1, 'relu')
        self.conv6 = Conv_Bn_Activation(base_channels * 16, base_channels * 8, 1, 1, 'relu')
        self.conv7 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        # UP
        self.upsample1 = Upsample(base_channels * 4, base_channels * 4)
        # R 85
        self.conv8 = Conv_Bn_Activation(base_channels * 4, base_channels * 4, 1, 1, 'relu')
        # R -1 -3
        self.conv9 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv10 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 1, 'relu')
        self.conv11 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv12 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 1, 'relu')
        self.conv13 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv14 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        # UP
        self.upsample2 = Upsample(base_channels * 2, base_channels * 2)
        # R 54
        self.conv15 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')
        # R -1 -3
        self.conv16 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        self.conv17 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 1, 'relu')
        self.conv18 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        self.conv19 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 1, 'relu')
        self.conv20 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')

    def forward(self, downsample4, downsample3, downsample2, downsample1):
        x0 = self.conv0(downsample4)
        # SPP
        m1 = self.maxpool1(x0)
        m2 = self.maxpool2(x0)
        m3 = self.maxpool3(x0)
        spp = torch.cat([m3, m2, m1, x0], dim=1)

        # SPP end
        x1 = self.conv1(spp)
        x1_1 = self.conv1_1(x1)
        x2 = self.conv2(x1_1)
        # UP
        up = self.upsample0(x2)

        x3 = self.conv3(downsample3)

        x3 = torch.cat([x3, up], dim=1)
        x4 = self.conv4(x3)
        x5 = self.conv5(x4)
        x6 = self.conv6(x5)
        x7 = self.conv7(x6)
        # UP
        up = self.upsample1(x7)
        # R 85
        x8 = self.conv8(downsample2)
        # R -1 -3
        x8 = torch.cat([x8, up], dim=1)

        x9 = self.conv9(x8)
        x10 = self.conv10(x9)
        x11 = self.conv11(x10)
        x12 = self.conv12(x11)
        x13 = self.conv13(x12)
        x14 = self.conv14(x13)

        # UP
        up = self.upsample2(x14)
        # R 54
        x15 = self.conv15(downsample1)
        # R -1 -3
        x15 = torch.cat([x15, up], dim=1)

        x16 = self.conv16(x15)
        x17 = self.conv17(x16)
        x18 = self.conv18(x17)
        x19 = self.conv19(x18)
        x20 = self.conv20(x19)
        return x20, x13, x6

class PAN(nn.Module):
    def __init__(self):
        super().__init__()

        self.conv1 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 1, 1, 'relu')
        self.conv2 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        # R -4
        self.conv3 = Conv_Bn_Activation(base_channels * 2, base_channels * 2, 3, 2, 'relu')

        # R -1 -16
        self.conv4 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        self.conv5 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 1, 'relu')
        self.conv6 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')
        self.conv7 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 1, 'relu')
        self.conv8 = Conv_Bn_Activation(base_channels * 4, base_channels * 2, 1, 1, 'relu')

        self.conv10 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        # R -4
        self.conv11 = Conv_Bn_Activation(base_channels * 2, base_channels * 4, 3, 2, 'relu')

        # R -1 -37
        self.conv12 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv13 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 1, 'relu')
        self.conv14 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv15 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 1, 'relu')
        self.conv16 = Conv_Bn_Activation(base_channels * 8, base_channels * 4, 1, 1, 'relu')
        self.conv17 = Conv_Bn_Activation(base_channels * 4, base_channels * 8, 3, 1, 'relu')

    def forward(self, input1, input2, input3):
        x1 = self.conv1(input1)
        x2 = self.conv2(input2)
        x3 = self.conv3(input1)
        # R -1 -16
        x3 = torch.cat([x3, x2], dim=1)
        x4 = self.conv4(x3)
        x5 = self.conv5(x4)
        x6 = self.conv6(x5)
        x7 = self.conv7(x6)
        x8 = self.conv8(x7)

        # R -4
        x10 = self.conv10(input3)
        x11 = self.conv11(x8)
        # R -1 -37
        x11 = torch.cat([x11, x10], dim=1)

        x12 = self.conv12(x11)
        x13 = self.conv13(x12)
        x14 = self.conv14(x13)
        x15 = self.conv15(x14)
        x16 = self.conv16(x15)
        x17 = self.conv17(x16)

        return x1, x17

class CSPDarknet53Small(nn.Module):
    def __init__(self, model_cfg, input_channels):
        super().__init__()
        self.model_cfg = model_cfg

        self.down1 = DownSample1(input_channels)
        self.down2 = DownSample2()
        self.down3 = DownSample3()
        self.down4 = DownSample4()

        self.neek = Neck()
        self.pan = PAN()

        self.num_bev_features = base_channels * 2

    def forward(self, spatial_features):
        x = spatial_features
        d1 = self.down1(x)
        d2 = self.down2(d1)
        d3 = self.down3(d2)
        d4 = self.down4(d3)

        x20, x13, x6 = self.neek(d4, d3, d2, d1)
        x1, x17  = self.pan(x20, x13, x6)

        return x1, x17