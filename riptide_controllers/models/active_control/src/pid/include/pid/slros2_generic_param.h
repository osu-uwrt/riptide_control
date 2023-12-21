/* Copyright 2022 The MathWorks, Inc. */

#ifndef _SLROS2_GENERIC_PARAM_H_
#define _SLROS2_GENERIC_PARAM_H_

#include "rclcpp/rclcpp.hpp"

/**
 * Base class for getting ROS 2 parameters in C++.
 *
 * This class is used by derived classes used for handling scalar and array
 * parameter values.
 */
class SimulinkParameterGetterBase {
  public:
    void initParam(const std::string& pName);
  protected:
    rclcpp::Node::SharedPtr nodePtr; ///< Pointer to node handle (node will be used to connect to parameter server)
    std::string paramName; ///< The name of the parameter
};


/**
 * Class for getting scalar ROS 2 parameters in C++.
 *
 * This class is used by code generated from the Simulink ROS 2
 * parameter blocks and is templatized by the expected C++ data type
 * for the parameter value.
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterGetter : public SimulinkParameterGetterBase {

  public:
    void setInitialValue(const CppParamType initValue);
    void getParameter(CppParamType* dataPtr);

  private:
    ROSCppParamType initialValue; ///< The default value that should be returned by getParameter
    CppParamType lastValidValue; ///< The last valid value that was received from the parameter server
};

/**
 * Set initial value for returned parameter value.
 *
 * This initial value will be returned if the parameter does not exist or does not have the correct
 * data type when the node is started.
 * @param[in] initValue The initial value.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterGetter<CppParamType, ROSCppParamType>::setInitialValue(
    const CppParamType initValue) {
    initialValue = initValue;
    lastValidValue = initValue;
    if (nodePtr->has_parameter(paramName)) {
        nodePtr->set_parameter(rclcpp::Parameter(paramName,rclcpp::ParameterValue(initialValue)));
    } else {
        nodePtr->declare_parameter<ROSCppParamType>(paramName,initialValue);
    }
}

/**
 * Get the value for a named parameter from the parameter server.
 * @param[out] dataPtr Pointer to initialized data variable. The retrieved parameter value will be
 * written to this location
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterGetter<CppParamType, ROSCppParamType>::getParameter(
    CppParamType* dataPtr) {
    rclcpp::Parameter prmValue;
    ROSCppParamType paramValue;

    // Get parameter as rclcpp::Parameter
    if (nodePtr->get_parameter(paramName, prmValue)) {
        try {
            paramValue = prmValue.get_value<ROSCppParamType>();
            // Cast the returned value into the data type that Simulink is expecting
            *dataPtr = static_cast<CppParamType>(paramValue);
            lastValidValue = *dataPtr;
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_WARN(nodePtr->get_logger(),"Dynamic type change for parameter, %s is not allowed. %s\n",paramName.c_str(),ex.what());
            *dataPtr = lastValidValue;
        }
    }
}



/**
 * Class for getting array ROS 2 parameters in C++.
 *
 * This class is used by code generated from the Simulink ROS 2
 * parameter blocks.
 * Note that the ROSCppParamType template parameter needs to refer to a container
 * type that supports the following operations:
 * - resize
 * - std::copy
 * std::string (used for string parameters) and std::vector (used for numeric arrays)
 * fall into this category.
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterArrayGetter : public SimulinkParameterGetterBase {

  public:
    void setInitialValue(const CppParamType* initValue, const uint32_t lengthToWrite);
    void getParameter(const uint32_t maxLength,
                          CppParamType* dataPtr,
                          uint32_t* receivedLength);

  private:
    ROSCppParamType initialValue; ///< The default value that should be returned by getParameter if
                                  ///< one of the error conditions occurs
    ROSCppParamType
        lastValidValue; ///< The last valid value that was received from the parameter server
};


/**
 * Set initial value for returned parameter value.
 *
 * This initial value will be returned if the parameter does not exist or does not have the correct
 * data type when the node is started.
 * @param[in] initValue The initial value.
 * @param[in] lengthToWrite The number of elements in the @c initValue array. Since the array is
 * passed as a pointer, the @c lengthToWrite argument is required to indicate how many elements the
 * array has.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterArrayGetter<CppParamType, ROSCppParamType>::setInitialValue(
    const CppParamType* desiredInitialValue,
    const uint32_t lengthToWrite) {
    initialValue.resize(lengthToWrite);

    // Store the initial value in a member variable. Note that std::copy will work on any
    // Iterable array, e.g. std::string or std::vector
    std::copy(desiredInitialValue, desiredInitialValue + lengthToWrite, initialValue.begin());
    if (nodePtr->has_parameter(paramName)) {
        nodePtr->set_parameter(rclcpp::Parameter(paramName,rclcpp::ParameterValue(initialValue)));
    } else {
        nodePtr->declare_parameter<ROSCppParamType>(paramName,initialValue);
    }
    
}


/**
 * Get the value for a named parameter from the parameter server.
 * @param[in] maxLength The maximum length of the returned array (in elements). The array in @c
 * dataPtr will have this many elements.
 * @param[out] dataPtr Pointer to initialized data array. The retrieved parameter value will be
 * written to this location
 * @param[out] receivedLength The actual number of array elements that was received. This value will
 * be <= than @c maxLength.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterArrayGetter<CppParamType, ROSCppParamType>::getParameter(
    const uint32_t maxLength,
    CppParamType* dataPtr,
    uint32_t* receivedLength) {

    rclcpp::Parameter prmValue;
    ROSCppParamType retrievedValue;
    // Get parameter as rclcpp::Parameter
    if (nodePtr->get_parameter(paramName, prmValue)) {
        try {
            retrievedValue = prmValue.get_value<ROSCppParamType>();
            // Cast the returned value into the data type that Simulink is expecting
            if (maxLength < retrievedValue.size()) {
                RCLCPP_WARN(nodePtr->get_logger(),"Value of ROS 2 parameter, '%s', can have a maximum of %d items, but value received from parameter server has %d items.\nThe output value will be truncated to first %d items.",
                           paramName.c_str(), maxLength, retrievedValue.size(),maxLength);
            }
            uint32_t copyLength = std::min(maxLength, static_cast<uint32_t>(retrievedValue.size()));
            std::copy(retrievedValue.begin(), retrievedValue.begin() + copyLength, dataPtr);
            *receivedLength = copyLength;
            
            // Remember last valid value
            lastValidValue.resize(copyLength);
            std::copy(retrievedValue.begin(), retrievedValue.begin() + copyLength,
                      lastValidValue.begin());
        } catch (rclcpp::ParameterTypeException::exception &ex) {
            RCLCPP_WARN(nodePtr->get_logger(),"Data-type for parameter, '%s', was changed at runtime. %s\n",paramName.c_str(),ex.what());
            std::copy(lastValidValue.begin(), lastValidValue.begin() + lastValidValue.size(),
                      dataPtr);            
            *receivedLength = uint32_t(lastValidValue.size());           
        }
    }
}


#endif
