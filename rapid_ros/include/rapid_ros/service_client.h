#ifndef _RAPID_ROS_SERVICE_CLIENT_H_
#define _RAPID_ROS_SERVICE_CLIENT_H_

#include "ros/service_client.h"

namespace rapid_ros {
// DECLARATIONS ---------------------------------------------------------------
template <class Service>
class ServiceClientInterface {
 public:
  virtual ~ServiceClientInterface() {}
  virtual bool call(Service& srv) = 0;
  virtual bool call(typename Service::Request& req,
                    typename Service::Response& res) = 0;
};

// Wraps a real ros::ServiceClient.
template <class Service>
class ServiceClient : public ServiceClientInterface<Service> {
 public:
  explicit ServiceClient(const ros::ServiceClient& client);
  virtual bool call(Service& srv);
  virtual bool call(typename Service::Request& req,
                    typename Service::Response& res);

 private:
  ServiceClient();
  ros::ServiceClient client_;
};

// A mock service client.
//
// Currently only works to test methods that call the service exactly once.
// A TODO would be to set different responses for different requests.
//
// All calls are successes unless set_success(false) is called.
template <class Service>
class MockServiceClient : public ServiceClientInterface<Service> {
 public:
  MockServiceClient();
  virtual bool call(Service& srv);
  virtual bool call(typename Service::Request& req,
                    typename Service::Response& res);

  // Mock methods.
  void set_success(bool success);
  void set_response(const typename Service::Response& response);

 private:
  bool success_;
  typename Service::Response response_;
};

// DEFINITIONS ----------------------------------------------------------------
// An interface for service clients.
template <class Service>
ServiceClient<Service>::ServiceClient(const ros::ServiceClient& client)
    : client_(client) {}

template <class Service>
bool ServiceClient<Service>::call(Service& srv) {
  return client_.call(srv);
}

template <class Service>
bool ServiceClient<Service>::call(typename Service::Request& req,
                                  typename Service::Response& res) {
  return client_.call(req, res);
}

template <class Service>
MockServiceClient<Service>::MockServiceClient()
    : success_(true), response_() {}

template <class Service>
bool MockServiceClient<Service>::call(Service& srv) {
  return call(srv.request, srv.response);
  // if (!success_) {
  //  return false;
  //}
  // srv.response = response_;
  // return true;
}

template <class Service>
bool MockServiceClient<Service>::call(typename Service::Request& req,
                                      typename Service::Response& res) {
  if (!success_) {
    return false;
  }
  res = response_;
  return true;
}

template <class Service>
void MockServiceClient<Service>::set_success(bool success) {
  success_ = success;
}

template <class Service>
void MockServiceClient<Service>::set_response(
    const typename Service::Response& response) {
  response_ = response;
}
}  // namespace rapid_ros

#endif  // _RAPID_ROS_SERVICE_CLIENT_H_
