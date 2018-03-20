#include "rapid_ros/params.h"

#include <limits.h>
#include <iostream>
#include <map>
#include <string>

#include "XmlRpcValue.h"
#include "gtest/gtest.h"
#include "rapid_testing/rosout_test_helper.h"
#include "ros/ros.h"

namespace rapid {
TEST(ParamsTest, TestGetStringOrThrowSuccess) {
  ros::param::set("~test_string", "hello world");
  try {
    std::string result = GetStringParamOrThrow("~test_string");
    EXPECT_EQ("hello world", result);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_string");
    FAIL();
  }
  ros::param::del("~test_string");
}

TEST(ParamsTest, TestGetStringOrThrowFailure) {
  ros::param::del("~test_string");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetStringParamOrThrow("~test_string"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetDoubleOrThrowSuccess) {
  ros::param::set("~test_double", std::numeric_limits<double>::max());
  try {
    double result = GetDoubleParamOrThrow("~test_double");
    EXPECT_EQ(std::numeric_limits<double>::max(), result);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_double");
    FAIL();
  }
  ros::param::del("~test_double");
}

TEST(ParamsTest, TestGetDoubleOrThrowFailure) {
  ros::param::del("~test_double");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetDoubleParamOrThrow("~test_double"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetFloatOrThrowSuccess) {
  ros::param::set("~test_float", std::numeric_limits<float>::max());
  try {
    float result = GetFloatParamOrThrow("~test_float");
    EXPECT_EQ(std::numeric_limits<float>::max(), result);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_float");
    FAIL();
  }
  ros::param::del("~test_float");
}

TEST(ParamsTest, TestGetFloatOrThrowFailure) {
  ros::param::del("~test_float");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetFloatParamOrThrow("~test_float"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetIntOrThrowSuccess) {
  ros::param::set("~test_int", std::numeric_limits<int>::max());
  try {
    int result = GetIntParamOrThrow("~test_int");
    EXPECT_EQ(std::numeric_limits<int>::max(), result);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_int");
    FAIL();
  }
  ros::param::del("~test_int");
}

TEST(ParamsTest, TestGetIntOrThrowFailure) {
  ros::param::del("~test_int");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetIntParamOrThrow("~test_int"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetBoolOrThrowSuccess) {
  ros::param::set("~test_bool", false);
  try {
    bool result = GetBoolParamOrThrow("~test_bool");
    EXPECT_EQ(false, result);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_bool");
    FAIL();
  }
  ros::param::del("~test_bool");
}

TEST(ParamsTest, TestGetBoolOrThrowFailure) {
  ros::param::del("~test_bool");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetBoolParamOrThrow("~test_bool"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetXmlRpcOrThrowSuccess) {
  XmlRpc::XmlRpcValue expected("hello world");
  ros::param::set("~test_xmlrpc", expected);
  try {
    XmlRpc::XmlRpcValue result = GetXmlRpcParamOrThrow("~test_xmlrpc");
    EXPECT_EQ("hello world", std::string(result));
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_xmlrpc");
    FAIL();
  }
  ros::param::del("~test_xmlrpc");
}

TEST(ParamsTest, TestGetXmlRpcOrThrowFailure) {
  ros::param::del("~test_xmlrpc");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetXmlRpcParamOrThrow("~test_xmlrpc"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetStringVectorOrThrowSuccess) {
  std::vector<std::string> expected;
  expected.push_back("hello world");
  ros::param::set("~test_string_vector", expected);
  try {
    std::vector<std::string> result =
        GetStringVectorParamOrThrow("~test_string_vector");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ("hello world", result[0]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_string_vector");
    FAIL();
  }
  ros::param::del("~test_string_vector");
}

TEST(ParamsTest, TestGetStringVectorOrThrowFailure) {
  ros::param::del("~test_string_vector");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetStringVectorParamOrThrow("~test_string_vector"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetDoubleVectorOrThrowSuccess) {
  std::vector<double> expected;
  expected.push_back(1.0);
  ros::param::set("~test_double_vector", expected);
  try {
    std::vector<double> result =
        GetDoubleVectorParamOrThrow("~test_double_vector");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(1.0, result[0]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_double_vector");
    FAIL();
  }
  ros::param::del("~test_double_vector");
}

TEST(ParamsTest, TestGetDoubleVectorOrThrowFailure) {
  ros::param::del("~test_double_vector");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetDoubleVectorParamOrThrow("~test_double_vector"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetFloatVectorOrThrowSuccess) {
  std::vector<float> expected;
  expected.push_back(1.0f);
  ros::param::set("~test_float_vector", expected);
  try {
    std::vector<float> result =
        GetFloatVectorParamOrThrow("~test_float_vector");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(1.0f, result[0]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_float_vector");
    FAIL();
  }
  ros::param::del("~test_float_vector");
}

TEST(ParamsTest, TestGetFloatVectorOrThrowFailure) {
  ros::param::del("~test_float_vector");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetFloatVectorParamOrThrow("~test_float_vector"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetIntVectorOrThrowSuccess) {
  std::vector<int> expected;
  expected.push_back(1);
  ros::param::set("~test_int_vector", expected);
  try {
    std::vector<int> result = GetIntVectorParamOrThrow("~test_int_vector");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(1, result[0]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_int_vector");
    FAIL();
  }
  ros::param::del("~test_int_vector");
}

TEST(ParamsTest, TestGetIntVectorOrThrowFailure) {
  ros::param::del("~test_int_vector");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetIntVectorParamOrThrow("~test_int_vector"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetBoolVectorOrThrowSuccess) {
  std::vector<int> expected;
  expected.push_back(false);
  ros::param::set("~test_bool_vector", expected);
  try {
    std::vector<bool> result = GetBoolVectorParamOrThrow("~test_bool_vector");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(false, result[0]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_bool_vector");
    FAIL();
  }
  ros::param::del("~test_bool_vector");
}

TEST(ParamsTest, TestGetBoolVectorOrThrowFailure) {
  ros::param::del("~test_bool_vector");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetBoolVectorParamOrThrow("~test_bool_vector"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetStringMapOrThrowSuccess) {
  std::map<std::string, std::string> expected;
  expected["hello"] = "world";
  ros::param::set("~test_string_map", expected);
  try {
    std::map<std::string, std::string> result =
        GetStringMapParamOrThrow("~test_string_map");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ("world", result["hello"]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_string_map");
    FAIL();
  }
  ros::param::del("~test_string_map");
}

TEST(ParamsTest, TestGetStringMapOrThrowFailure) {
  ros::param::del("~test_string_map");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetStringMapParamOrThrow("~test_string_map"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetDoubleMapOrThrowSuccess) {
  std::map<std::string, double> expected;
  expected["test"] = -std::numeric_limits<double>::max();
  ros::param::set("~test_double_map", expected);
  try {
    std::map<std::string, double> result =
        GetDoubleMapParamOrThrow("~test_double_map");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(-std::numeric_limits<double>::max(), result["test"]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_double_map");
    FAIL();
  }
  ros::param::del("~test_double_map");
}

TEST(ParamsTest, TestGetDoubleMapOrThrowFailure) {
  ros::param::del("~test_double_map");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetDoubleMapParamOrThrow("~test_double_map"),
               std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetFloatMapOrThrowSuccess) {
  std::map<std::string, float> expected;
  expected["test"] = -std::numeric_limits<float>::max();
  ros::param::set("~test_float_map", expected);
  try {
    std::map<std::string, float> result =
        GetFloatMapParamOrThrow("~test_float_map");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(-std::numeric_limits<float>::max(), result["test"]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_float_map");
    FAIL();
  }
  ros::param::del("~test_float_map");
}

TEST(ParamsTest, TestGetFloatMapOrThrowFailure) {
  ros::param::del("~test_float_map");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetFloatMapParamOrThrow("~test_float_map"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetIntMapOrThrowSuccess) {
  std::map<std::string, int> expected;
  expected["test"] = -std::numeric_limits<int>::max();
  ros::param::set("~test_int_map", expected);
  try {
    std::map<std::string, int> result = GetIntMapParamOrThrow("~test_int_map");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(-std::numeric_limits<int>::max(), result["test"]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_int_map");
    FAIL();
  }
  ros::param::del("~test_int_map");
}

TEST(ParamsTest, TestGetIntMapOrThrowFailure) {
  ros::param::del("~test_int_map");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetIntMapParamOrThrow("~test_int_map"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}

TEST(ParamsTest, TestGetBoolMapOrThrowSuccess) {
  std::map<std::string, bool> expected;
  expected["test"] = false;
  ros::param::set("~test_bool_map", expected);
  try {
    std::map<std::string, bool> result =
        GetBoolMapParamOrThrow("~test_bool_map");
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(false, result["test"]);
  } catch (const std::runtime_error& e) {
    ros::param::del("~test_bool_map");
    FAIL();
  }
  ros::param::del("~test_bool_map");
}

TEST(ParamsTest, TestGetBoolMapOrThrowFailure) {
  ros::param::del("~test_bool_map");
  RosoutTestHelper rosout;
  rosout.Start();
  EXPECT_THROW(GetBoolMapParamOrThrow("~test_bool_map"), std::runtime_error);
  rosout.WaitForMessageCount(1, ros::Duration(1.0));
  EXPECT_TRUE(rosout.WasErrorPublished());
}
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rapid_ros_params_test");
  ros::NodeHandle nh;
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
