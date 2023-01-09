#pragma once


#include "ros/ros.h"
#include <map>
#include <algorithm>
#include "sensor_msgs/Imu.h" 
#include "dmapping/scanType.h"
#include "pcl_conversions/pcl_conversions.h"
#include <dmapping/utility.h>
#include "math.h"


/* A steam of time stamped data  for lookup*/
namespace dmapping {

typedef std::pair<double, Eigen::Quaterniond> stampedImu;

bool compare (const stampedImu i, const stampedImu& j);

class ImuHandler{

public:

  ImuHandler(){}

  void AddMsg(sensor_msgs::Imu::ConstPtr msg);

  bool Get(const double& tStamp, Eigen::Quaterniond& data);

  bool TimeContained(const double);

  std::vector<stampedImu>::iterator end() {return data_.end();}

  std::vector<stampedImu>::iterator begin() {return data_.begin();}

  std::size_t size(){return data_.size();}

private:

  void Add(sensor_msgs::Imu& msg);

  std::vector<stampedImu> data_;

  double first = 0;


};



class Scan{

public:

  Scan(Cloud::Ptr cloudInput);

  Cloud::Ptr GetCloud();

  double GetStamp() const;

  Cloud::Ptr cloud_;
  double stamp_;

};

class ScanHandler{

public:

  ScanHandler(){}

  void AddMsg(sensor_msgs::PointCloud2::ConstPtr laserCloudMsg);

  std::vector<Scan>::iterator end() { return data_.end(); }

  std::vector<Scan>::iterator begin() { return data_.begin(); }

  size_t size() {return data_.size();}

  std::vector<Scan> data_;

private:

  void Add(Cloud::Ptr cloud);

};

/*
class StampedData
{
  typedef std::pair<double,> stampedVal;

  bool compare (const stampedVal i,const stampedVal& j) { return (i.first < j.first); }

public:
  StampedData(const size_t sizeHistory = 0) : sizeHistory_(sizeHistory) {}

  void Add(const double& stamp, const T& val){
    data_.push_back(std::make_pair(stamp,val));
    if(sizeHistory_ != 0 && data_.size() > sizeHistory_){
      data_.erase(data_.begin());
    }
  }

  bool Get(const double& tStamp, T& data){
    auto first = data_.begin();
    auto last = data_.end();
    stampedVal search(tStamp,data);
    if ( std::binary_search (first, last, search,  std::bind( &StampedData<T>::compare(), this)) ){
      cout << "found: idx:" << std::distance(data_.begin(), first) << ", val: " << first->first <<", t diff" << tStamp - first->first << endl;
      data = first->second;
      return true;
    }
    else
      return false;
  }

  std::vector<stampedVal> data_;
  const size_t sizeHistory_;

};

*/

/*

typedef uint32_t nsec_t;
typedef uint32_t sec_t;

template <class T>
class StampedData
{
  typedef std::map<nsec_t, T> batcnNsec;
  typedef std::map<sec_t,  batcnNsec> batcnSec;

  struct Iterator
      {
          using iterator_category = std::forward_iterator_tag;
          using difference_type   = std::ptrdiff_t;
          using value_type        = T;
          using pointer           = T*;
          using reference         = T&;

          Iterator(pointer ptr) : m_ptr(ptr) {}

          reference operator*() const { return *m_ptr; }
          pointer operator->() { return m_ptr; }
          Iterator& operator++() { m_ptr++; return *this; }
          Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
          friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
          friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };

      private:
          pointer m_ptr;
      };

public:
  StampedData(const batcnSec tHistory) : tHistory_(tHistory) {}

  void Add(const T& data, const ros::Time& tStamp);

  bool Get(const ros::Time& tStamp,T& data, const bool interpolate = true);

protected:
  Iterator begin();
  Iterator end();

protected:
  const batcnSec tHistory_;
  batcnSec bucket_;

};

*/

}


