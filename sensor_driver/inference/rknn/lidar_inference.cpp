#ifdef HAVE_RKNN_ENABLE

#include "lidar_inference.h"
#include <iostream>
#include <queue>
#include <unordered_map>

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i)
    {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }

    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
            "type=%s, qnt_type=%s, "
            "zp=%d, scale=%f\n",
            attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
            attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
            get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

LidarInference::LidarInference(LidarEngineParameter parameter)
{
    parameter_ = parameter;
    LOG_INFO("Loading RKNN object detection model...");
    int model_data_size = 0;
    unsigned char *model_data = load_model(parameter.rknn_file.c_str(), &model_data_size);
    int ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0)
    {
        LOG_ERROR("rknn_init error ret={}", ret);
        return;
    }

    ret = rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1_2);
    if (ret < 0) {
        LOG_WARN("rknn_set_core_mask error ret={}", ret);
    }

    // version
    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0)
    {
        LOG_ERROR("rknn_query(RKNN_QUERY_SDK_VERSION) error ret={}", ret);
        return;
    }
    LOG_INFO("sdk version: {}, driver version: {}", version.api_version, version.drv_version);

    // input /output
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0)
    {
        LOG_ERROR("rknn_query(RKNN_QUERY_IN_OUT_NUM) error ret={}", ret);
        return;
    }
    LOG_INFO("model input num: {}, output num: {}", io_num.n_input, io_num.n_output);

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0)
        {
            LOG_ERROR("rknn_query(RKNN_QUERY_INPUT_ATTR) error ret={}", ret);
            return;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        dump_tensor_attr(&(output_attrs[i]));
        if (strcmp(output_attrs[i].name, "batch_cls_preds") == 0) {
            feature_stride_ = std::sqrt((parameter_.grid_size[0] * parameter_.grid_size[1]) / output_attrs[i].dims[0]);
            num_class_ = output_attrs[i].dims[1];
        }
    }
    printf("  feature_stride=%d, num_class=%d\n", feature_stride_, num_class_);

    // meshgrid
    for(int h = 0; h < (parameter_.grid_size[1] / feature_stride_); h++) {
        for (int w = 0; w < (parameter_.grid_size[0] / feature_stride_); w++) {
            xs_.push_back(w);
            ys_.push_back(h);
        }
    }

    for (int i = 0; i < 64; i++) {
        points_num_map_[i] = std::log(i + 1) / std::log(64);
    }

    input_data_size_ = parameter_.grid_size[0] * parameter_.grid_size[1] * parameter_.grid_size[2] * sizeof(half);
    points_num_size_ = parameter_.grid_size[0] * parameter_.grid_size[1] * parameter_.grid_size[2] * sizeof(uint8_t);

    input_data_ = (half *) malloc(input_data_size_);
    points_num_ = (uint8_t *) malloc(points_num_size_);
    valid_ = true;
}

LidarInference::~LidarInference()
{
    free(input_data_);
    free(points_num_);
}

void LidarInference::reset()
{

}

int LidarInference::forward(const float* points, int point_num, const float* motion, bool runtime)
{
    if (!valid_) {
      return -1;
    }
    // preprocess
    pre_process(points, point_num);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_FLOAT16;
    inputs[0].size = input_data_size_;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 1;
    inputs[0].buf = input_data_;
    rknn_inputs_set(ctx, io_num.n_input, inputs);

    int ret = rknn_run(ctx, NULL);
    return 0;
}

void LidarInference::get_output(float* cls_preds, float* box_preds, int* label_preds, float* freespace) {
    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        outputs[i].want_float = 0;
    }
    int ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

    // post process
    post_process(reinterpret_cast<half*>(outputs[0].buf), reinterpret_cast<half*>(outputs[1].buf), cls_preds, box_preds, label_preds);
}

void LidarInference::pre_process(const float* points, int point_num)
{
    memset(input_data_, 0, input_data_size_);
    memset(points_num_, 0, points_num_size_);
    for (int i = 0; i < point_num; i++) {
        int idx = std::floor((points[parameter_.num_feature * i + 0] - parameter_.pointcloud_range[0]) / parameter_.voxel_size[0]);
        int idy = std::floor((points[parameter_.num_feature * i + 1] - parameter_.pointcloud_range[1]) / parameter_.voxel_size[1]);
        float pz = (points[parameter_.num_feature * i + 2] - parameter_.pointcloud_range[2]) / (parameter_.pointcloud_range[5] - parameter_.pointcloud_range[2]);
        if ((idx < 0 || idx >= parameter_.grid_size[0])) {
            continue;
        }
        if ((idy < 0 || idy >= parameter_.grid_size[1])) {
            continue;
        }
        if ((pz < 0 || pz > 1.0f)) {
            continue;
        }

        unsigned int offset = (idy * parameter_.grid_size[0] + idx) * parameter_.grid_size[2];
        half pzh = half(pz);
        half pih = half(points[parameter_.num_feature * i + 3]);
        if (input_data_[offset + 1] < pzh) {
            input_data_[offset + 0] = pih;
            input_data_[offset + 1] = pzh;
        }
        if (points_num_[offset] < (uint8_t)64) {
            points_num_[offset]++;
            input_data_[offset + 2] = half(points_num_map_[points_num_[offset]]);
        }
    }
}

using Proposal = std::pair<float, std::vector<int>>;
void LidarInference::post_process(half *heatmap, half *box_feat, float* cls_preds, float* box_preds, int* label_preds)
{
    // Top K
    std::priority_queue<Proposal, std::vector<Proposal>, std::greater<Proposal>> q;
    int n = parameter_.grid_size[0] * parameter_.grid_size[1] / feature_stride_ / feature_stride_;
    for (int i = 0; i < n; i++) {
        float score_max = 0;
        int   class_max = 0;
        for (int j = 0; j < num_class_; j++) {
            float score = float(heatmap[num_class_ * i + j]) * std::min(1.0f, std::max(0.0f, float(box_feat[9 * i + 8])));
            if (score_max < score) {
                score_max = score;
                class_max = j;
            }
        }
        if(q.size() < parameter_.num_proposal) {
            if (score_max >= 0.1) {
                q.push(Proposal(score_max, {i, class_max}));
            }
        } else if(q.top().first < score_max) {
            q.pop();
            q.push(Proposal(score_max, {i, class_max}));
        }
    }

    int num_proposal = std::min(parameter_.num_proposal, int(q.size()));
    for (int i = 0; i < num_proposal; i++) {
        int src = q.top().second[0];
        int dst = num_proposal - i - 1;

        cls_preds[dst] = q.top().first;
        label_preds[dst] = q.top().second[1] + 1;
        box_preds[7 * dst + 0] = (xs_[src] + float(box_feat[9 * src + 0])) * feature_stride_ * parameter_.voxel_size[0] + parameter_.pointcloud_range[0];
        box_preds[7 * dst + 1] = (ys_[src] + float(box_feat[9 * src + 1])) * feature_stride_ * parameter_.voxel_size[1] + parameter_.pointcloud_range[1];
        box_preds[7 * dst + 2] = float(box_feat[9 * src + 2]);
        box_preds[7 * dst + 3] = std::exp(float(box_feat[9 * src + 3]));
        box_preds[7 * dst + 4] = std::exp(float(box_feat[9 * src + 4]));
        box_preds[7 * dst + 5] = std::exp(float(box_feat[9 * src + 5]));
        box_preds[7 * dst + 6] = std::atan2(float(box_feat[9 * src + 6]), float(box_feat[9 * src + 7]));
        q.pop();
    }
}

std::shared_ptr<LidarInference> create_engine(LidarEngineParameter parameter) {
  std::shared_ptr<LidarInference> engine = std::shared_ptr<LidarInference>(new LidarInference(parameter));
  return engine;
}

#endif // HAVE_RKNN_ENABLE