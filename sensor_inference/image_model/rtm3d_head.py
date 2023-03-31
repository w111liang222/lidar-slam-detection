from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import torch
import torch.nn as nn
import torch.nn.functional as F
import copy

class Sequential(torch.nn.Module):
    r"""A sequential container.
    Modules will be added to it in the order they are passed in the constructor.
    Alternatively, an ordered dict of modules can also be passed in.

    To make it easier to understand, given is a small example::

        # Example of using Sequential
        model = Sequential(
                  nn.Conv2d(1,20,5),
                  nn.ReLU(),
                  nn.Conv2d(20,64,5),
                  nn.ReLU()
                )

        # Example of using Sequential with OrderedDict
        model = Sequential(OrderedDict([
                  ('conv1', nn.Conv2d(1,20,5)),
                  ('relu1', nn.ReLU()),
                  ('conv2', nn.Conv2d(20,64,5)),
                  ('relu2', nn.ReLU())
                ]))

        # Example of using Sequential with kwargs(python 3.6+)
        model = Sequential(
                  conv1=nn.Conv2d(1,20,5),
                  relu1=nn.ReLU(),
                  conv2=nn.Conv2d(20,64,5),
                  relu2=nn.ReLU()
                )
    """

    def __init__(self, *args, **kwargs):
        super(Sequential, self).__init__()
        if len(args) == 1 and isinstance(args[0], OrderedDict):
            for key, module in args[0].items():
                self.add_module(key, module)
        else:
            for idx, module in enumerate(args):
                self.add_module(str(idx), module)
        for name, module in kwargs.items():
            if sys.version_info < (3, 6):
                raise ValueError("kwargs only supported in py36+")
            if name in self._modules:
                raise ValueError("name exists.")
            self.add_module(name, module)

    def __getitem__(self, idx):
        if not (-len(self) <= idx < len(self)):
            raise IndexError("index {} is out of range".format(idx))
        if idx < 0:
            idx += len(self)
        it = iter(self._modules.values())
        for i in range(idx):
            next(it)
        return next(it)

    def __len__(self):
        return len(self._modules)

    def add(self, module, name=None):
        if name is None:
            name = str(len(self._modules))
            if name in self._modules:
                raise KeyError("name exists")
        self.add_module(name, module)

    def forward(self, input):
        # i = 0
        for module in self._modules.values():
            # print(i)
            input = module(input)
            # i += 1
        return input

class SepHead(nn.Module):
    def __init__(
        self,
        in_channels,
        heads,
        head_conv=64,
        final_kernel=1,
        bn=False,
        init_bias=-2.19,
        **kwargs,
    ):
        super(SepHead, self).__init__(**kwargs)

        self.heads = heads
        for head in self.heads:
            classes, num_conv = self.heads[head]

            fc = Sequential()
            for i in range(num_conv-1):
                fc.add(nn.Conv2d(in_channels, head_conv,
                    kernel_size=final_kernel, stride=1,
                    padding=final_kernel // 2, bias=True))
                if bn:
                    fc.add(nn.BatchNorm2d(head_conv))
                fc.add(nn.ReLU())

            fc.add(nn.Conv2d(head_conv, classes,
                    kernel_size=final_kernel, stride=1,
                    padding=final_kernel // 2, bias=True))

            if 'hm' in head:
                fc[-1].bias.data.fill_(init_bias)

            self.__setattr__(head, fc)


    def forward(self, x):
        ret_dict = dict()
        for head in self.heads:
            ret_dict[head] = self.__getattr__(head)(x)

        return ret_dict

class RTM3DHEAD(nn.Module):
    def __init__(self, model_cfg, image_shape, downscale, num_cls, input_channels):
        super(RTM3DHEAD, self).__init__()
        self.model_cfg = model_cfg
        self.num_cls = num_cls
        self.H = int(image_shape[0] / downscale)
        self.W = int(image_shape[1] / downscale)

        share_conv_channel=64
        num_hm_conv=2
        common_heads = model_cfg.COMMON_HEADS
        init_bias=-2.19
        # a shared convolution
        self.shared_conv = nn.Sequential(
            nn.Conv2d(input_channels, share_conv_channel,
            kernel_size=3, padding=1, bias=True),
            nn.BatchNorm2d(share_conv_channel),
            nn.ReLU(inplace=True)
        )

        heads = copy.deepcopy(common_heads)
        heads.update(dict(hm=(self.num_cls, num_hm_conv)))
        self.task = SepHead(share_conv_channel, heads, bn=True, init_bias=init_bias, final_kernel=3)


    def forward(self, x):
        x = self.shared_conv(x)
        data_dict = self.task(x)
        hm = data_dict['hm'].sigmoid_()
        hps = data_dict['hps']
        rot = data_dict['rot']
        dim = data_dict['dim']

        H = self.H
        W = self.W
        K = self.model_cfg.NMS_PRE_MAXSIZE
        num_cls = self.num_cls

        hm = hm.permute(0, 2, 3, 1).contiguous().reshape(H*W, num_cls)
        hps = hps.permute(0, 2, 3, 1).contiguous().reshape(H*W, 18)
        dim = dim.permute(0, 2, 3, 1).contiguous().reshape(H*W, 3)
        rot = rot.permute(0, 2, 3, 1).contiguous().reshape(H*W, 2)

        score_preds, label_preds = torch.max(hm, dim=-1)
        score_preds, indices = torch.topk(score_preds, k=K)
        label_preds = label_preds[indices]

        hm = hm.view(H, W, num_cls)
        kps = hps[indices]
        dim = dim[indices]
        rot = rot[indices]

        return hm, kps, dim, rot, score_preds, label_preds, indices
