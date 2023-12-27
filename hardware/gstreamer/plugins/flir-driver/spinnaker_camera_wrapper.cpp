#include "spinnaker_camera_wrapper.h"

#include <spdlog/spdlog.h>

void SpinnakerCameraWrapper::setEnumValue(string setting, string value)
{
    INodeMap &nodeMap = pcam->GetNodeMap();

    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        SPDLOG_ERROR("Unable to set {} to {} (enum retrieval). Aborting...", setting, value);

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        SPDLOG_ERROR("Unable to set {} to {} (entry retrieval). Aborting...", setting, value);

    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();

    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);
}
string SpinnakerCameraWrapper::getEnumValue(string setting) const
{
    INodeMap &nodeMap = pcam->GetNodeMap();
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    CEnumEntryPtr ptrValue = ptr->GetCurrentEntry();
    return ptrValue->ToString().c_str();
}

void SpinnakerCameraWrapper::setIntValue(string setting, int value)
{
    INodeMap &nodeMap = pcam->GetNodeMap();

    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
    {
        SPDLOG_ERROR("Unable to set {} to {} (ptr retrieval). Aborting...", setting, value);
    }
    ptr->SetValue(value);
}
int SpinnakerCameraWrapper::getIntValue(string setting) const
{
    INodeMap &nodeMap = pcam->GetNodeMap();
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    return ptr->GetValue();
}

void SpinnakerCameraWrapper::setFloatValue(string setting, float value)
{
    INodeMap &nodeMap = pcam->GetNodeMap();

    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
    {
        SPDLOG_ERROR("Unable to set {} to {} (ptr retrieval). Aborting...", setting, value);
    }
    ptr->SetValue(value);
}
float SpinnakerCameraWrapper::getFloatValue(string setting) const
{
    INodeMap &nodeMap = pcam->GetNodeMap();
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    return ptr->GetValue();
}

void SpinnakerCameraWrapper::setBoolValue(string setting, bool value)
{
    INodeMap &nodeMap = pcam->GetNodeMap();

    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
    {
        SPDLOG_ERROR("Unable to set {} to {} (ptr retrieval). Aborting...", setting, value);
    }
    ptr->SetValue(value);
}
bool SpinnakerCameraWrapper::getBoolValue(string setting) const
{
    INodeMap &nodeMap = pcam->GetNodeMap();
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    return ptr->GetValue();
}

void SpinnakerCameraWrapper::setBufferMode(string value)
{
    INodeMap &nodeMap = pcam->GetTLStreamNodeMap();

    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode("StreamBufferHandlingMode");

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());

    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();

    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);
} // special node map

void SpinnakerCameraWrapper::setResolutionPixels(int width, int height)
{
    CIntegerPtr ptrHeight = pcam->GetNodeMap().GetNode("Height");
    CIntegerPtr ptrWidth = pcam->GetNodeMap().GetNode("Width");
    if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth))
    {
        SPDLOG_ERROR("Unable to set width. Aborting...");
        return;
    }
    int64_t widthMax = ptrWidth->GetMax();
    if (widthMax < width)
        width = widthMax;
    ptrWidth->SetValue(width);

    if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight))
    {
        SPDLOG_ERROR("Unable to set height. Aborting...");
        return;
    }
    int64_t heightMax = ptrHeight->GetMax();
    if (heightMax < height)
        height = heightMax;

    ptrHeight->SetValue(height);

} // for convinient and sanity check
