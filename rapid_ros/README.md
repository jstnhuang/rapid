# rapid_ros

rapid_ros contains wrappers around common ROS functionality.
Note that unlike other rapid packages, the namespace is `rapid_ros`, not `rapid::ros`.
This is for convenience, otherwise, the `ros` namespace is interpreted as `::rapid::ros` instead of `::ros` inside of other rapid code.

Note that tests that use the `MockActionClient` should run as rostests not pure gtests, because it uses ROS logging.

## Publisher wrapper
`publisher.h` includes a `PublisherInterface`, which should be used instead of a `ros::Publisher` directly.
This enables us to write unit tests for classes that use publishers.

Real `Publisher` usage:
```cpp
ros::NodeHandle nh;
PublisherInterface<Marker>* marker_pub = new Publisher<Marker>(nh.advertise<Marker>("markers", 100));
marker_pub->publish(msg);
delete marker_pub;
```

`MockPublisher` usage:
```cpp
MockPublisher<Marker> pub;
PublisherInterface<Marker>* test_pub = &pub;
test_pub->publish(msg);
Marker last_message = pub.last_message();
vector<Marker> history = pub.sent_messages();
```

## Action client wrapper
`action_client.h` includes an `ActionClientInterface`, which should be used instead of an `actionlib::SimpleActionClient` directly.
This is because code using a `SimpleActionClient` cannot be unit-tested without creating a mock action server that runs concurrently with the class under test.
Our previous experience with this is that these tests will pass on a local development machine, but become flaky inside of Travis.
Not all functionality of `SimpleActionClient` is in the `ActionClientInterface` yet, but we hope to expand it as needed.

Real `ActionClient` usage:
```cpp
ActionClientInterface<MyAction>* client = new ActionClient<MyAction>("my_action");
client.waitForServer(ros::Duration(10));
client.sendGoal(goal);
client.waitForResult(ros::Duration(5));
MyAction::ResultConstPtr result = client.getResult();
```

`MockActionClient` usage:
```cpp
ActionClientInterface<MyAction>* client = new MockActionClient<MyAction>();
client.set_server_delay(ros::Duration(11));
client.waitForServer(ros::Duration(10)); // Will return false;
client.waitForServer(ros::Duration(12)); // Will return true;

client.set_result_delay(ros::Duration(6));
client.waitForResult(ros::Duration(7)); // Will return true;

client.set_result(the_result);
MyAction::ResultConstPtr result = client.getResult(); // result will be equal to the_result

client.sendGoal(goal);
MyAction::Goal goal = client.last_goal(); // Read the last goal that was sent.
```
