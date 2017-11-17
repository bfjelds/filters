/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTERS_FILTER_BASE_H_
#define FILTERS_FILTER_BASE_H_

#include "rclcpp/rclcpp.hpp"

#define ROS_ERROR(...)

namespace filters
{

/** \brief A Base filter class to provide a standard interface for all filters
 *
 */
template<typename T>
class FilterBase : rclcpp::Node
{
public:
  /** \brief Default constructor used by Filter Factories
   */
  FilterBase() : rclcpp::Node("filterbase"), configured_(false){};

  /** \brief Virtual Destructor
   */
  virtual ~FilterBase(){};

  /** \brief Update the filter and return the data seperately
   * This is an inefficient way to do this and can be overridden in the derived class
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   */
  virtual bool update(const T& data_in, T& data_out)=0;

  /** \brief Get the type of the filter as a string */
  std::string getType() {return filter_type_;};

  /** \brief Get the name of the filter as a string */
  inline const std::string& getName(){return filter_name_;};


protected:

  /** \brief Pure virtual function for the sub class to configure the filter
   * This function must be implemented in the derived class.
   */
  virtual bool configure()=0;


  /** \brief Get a filter parameter as a string 
   * \param name The name of the parameter
   * \param value The string to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string& name, std::string& value)
  {
    return false;
  }

  /** \brief Get a filter parameter as a boolean
   * \param name The name of the parameter
   * \param value The boolean to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string& name, bool& value)
  {
    return false;
  }

  /** \brief Get a filter parameter as a double
   * \param name The name of the parameter
   * \param value The double to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, double& value)
  {
    return false;
  }

  /** \brief Get a filter parameter as a int
   * \param name The name of the parameter
   * \param value The int to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, int& value)
  {
    return false;
  }

  /** \brief Get a filter parameter as an unsigned int
   * \param name The name of the parameter
   * \param value The int to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, unsigned  int& value)
  {
    return false;
  };

  /** \brief Get a filter parameter as a std::vector<double>
   * \param name The name of the parameter
   * \param value The std::vector<double> to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, std::vector<double>& value)
  {
    return false;
  }

  /** \brief Get a filter parameter as a std::vector<string>
   * \param name The name of the parameter
   * \param value The std::vector<sgring> to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, std::vector<std::string>& value)
  {
    return false;
  }

 
  ///The name of the filter
  std::string filter_name_;
  ///The type of the filter (Used by FilterChain for Factory construction)
  std::string filter_type_;
  /// Whether the filter has been configured.  
  bool configured_;

};


template <typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase():number_of_channels_(0){};
  
  virtual bool configure()=0;
  

  /** \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)=0;

  virtual bool update(const T& data_in, T& data_out)
  {
    ROS_ERROR("THIS IS A MULTI FILTER DON'T CALL SINGLE FORM OF UPDATE");
    return false;
  };


protected:
  using FilterBase<T>::configured_;
  using FilterBase<T>::filter_type_;
  using FilterBase<T>::filter_name_;

  /// How many parallel inputs for which the filter is to be configured
  unsigned int number_of_channels_;
  

};

}
#endif //#ifndef FILTERS_FILTER_BASE_H_
