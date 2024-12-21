#include "GetPklData.h"

namespace py = pybind11;

std::vector<std::string> getFiles(std::string src_dir)
{

    const char *ext;
    ext = ".pkl";
    std::vector<std::string> result;
    std::string m_ext(ext);

    char *src_d = (char *)(src_dir.data());
    if (src_dir == "")
    {
        src_d = (char *)"./";
    }

    DIR *dir = opendir(src_d);
    if (dir == NULL)
    {
        printf("[ERROR] %s is not a directory or not exist!\n", src_d);
        return result;
    }

    struct dirent *d_ent = NULL;
    char dot[3] = ".";
    char dotdot[6] = "..";

    while ((d_ent = readdir(dir)) != NULL)
    {
        if ((strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0))
        {
            if (d_ent->d_type != DT_DIR)
            {
                std::string d_name(d_ent->d_name);
                if (strcmp(d_name.c_str() + d_name.length() - m_ext.length(), m_ext.c_str()) == 0)
                {
                    result.push_back(std::string(d_ent->d_name));
                }
            }
        }
    }
    // sort the returned files
    sort(result.begin(), result.end());
    closedir(dir);
    return result;
}

py::dict getPklData(std::string fliename)
{
    // py::scoped_interpreter python;
    py::object open = py::module::import("builtins").attr("open");
    py::object pickle = py::module::import("pickle");
    py::object file = open(fliename, "rb");
    py::object data_dict = pickle.attr("load")(file);
    file.attr("close")();
    py::dict data = data_dict.cast<py::dict>();
    return data;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr toPclPointCloud(const py::array_t<float> &input)
{
    auto ref_input = input.unchecked<2>();
    auto point_num = ref_input.shape(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->width = static_cast<int>(point_num);
    cloud->height = 1;
    cloud->points.resize(point_num);
    for (int i = 0; i < point_num; i++)
    {
        cloud->points[i].x = ref_input(i, 0);
        cloud->points[i].y = ref_input(i, 1);
        cloud->points[i].z = ref_input(i, 2);
        cloud->points[i].intensity = ref_input(i, 3) * 255.0f;
    }
    return cloud;
}

cv::Mat toCvMatImage(py::bytes input)
{
    int n = py::len(input);

    cv::Mat outimage(1, n, CV_8UC1);

    int j = 0;
    for (auto i : input)
    {
        outimage.at<uchar>(0, j) = int(i.cast<int>());
        j++;
    }

    return outimage;
}

Ins_t toIns(py::dict input)
{
    Ins_t ins;
    ins.latitude = input["latitude"].cast<double>();
    ins.longitude = input["longitude"].cast<double>();
    ins.altitude = input["altitude"].cast<double>();
    ins.heading = input["heading"].cast<double>();
    ins.pitch = input["pitch"].cast<double>();
    ins.roll = input["roll"].cast<double>();
    ins.Ve = input["Ve"].cast<double>();
    ins.Vn = input["Vn"].cast<double>();
    ins.Vu = input["Vu"].cast<double>();
    if (input.contains("status")) {
        ins.status = input["status"].cast<int>();
    } else {
        ins.status = input["Status"].cast<int>();
    }
    ins.timestamp = input["timestamp"].cast<uint64_t>();
    return ins;
}