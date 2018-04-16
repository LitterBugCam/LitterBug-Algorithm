#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <iostream>
#include <fstream>
#include </usr/local/include/mqtt/Client.hpp>
 #include <NetworkConnection.hpp>

#include <errno.h>
#include <sys/types.h>
#include <sys/inotify.h>
#include <unistd.h>

using namespace awsiotsdk;
#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define EVENT_BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )

 int length, i = 0;
  int fd,   wd;
  char buffer[EVENT_BUF_LEN];
  
int main(int argc, char * argv[])
 {
	 
	 
	 //AWS IOT gateway initialization
	  std::shared_ptr<NetworkConnection> p_network_connection;

std::shared_ptr<MqttClient> p_client = MqttClient::Create(p_network_connection, std::chrono::milliseconds(30000));

rc = p_client->Connect(std::chrono::milliseconds(30000), false, mqtt::Version::MQTT_3_1_1, std::chrono::seconds(60), 
Utf8String::Create("<Litterbug_desktop>"), nullptr, nullptr, nullptr);
util::String p_topic_name_str = <topic>;
std::unique_ptr<Utf8String> p_topic_name = Utf8String::Create(p_topic_name_str);
mqtt::Subscription::ApplicationCallbackHandlerPtr p_sub_handler = std::bind(&<handler>, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
std::shared_ptr<mqtt::Subscription> p_subscription = mqtt::Subscription::Create(std::move(p_topic_name), mqtt::QoS::QOS0, p_sub_handler, nullptr);
util::Vector<std::shared_ptr<mqtt::Subscription>> topic_vector;
topic_vector.push_back(p_subscription);
rc = p_client->Subscribe(topic_vector, std::chrono::milliseconds(30000));


	 //inotify initialization
	  fd = inotify_init();
     if ( fd < 0 ) {
    perror( "inotify_init" );
	}
	   wd = inotify_add_watch( fd, "/home/ilias/litter_test/LitterDetection/detections", IN_CREATE | IN_DELETE );

	 
	 // AWS  initialization
    Aws::SDKOptions options;
    Aws::InitAPI(options);
     Aws::Client::ClientConfiguration clientConfig;
   clientConfig.region = Aws::Region::US_WEST_2;

Aws::S3::S3Client s3_client(clientConfig);
Aws::S3::Model::PutObjectRequest object_request;



object_request.WithBucket("littercam").WithKey("detected_litter.jpg");

  




// Binary files must also have the std::ios_base::bin flag or'ed in

 while(1)
  {
  length = read( fd, buffer, EVENT_BUF_LEN ); 
  /*checking for error*/
  if ( length < 0 ) {
    perror( "read" );
  }  
	
  /*actually read return the list of change events happens. Here, read the change event one by one and process it accordingly.*/
  while ( i < length )
  {
      struct inotify_event *event = ( struct inotify_event * ) &buffer[ i ];
  if ( event->len ) {
      if ( event->mask & IN_CREATE || event->mask & IN_MODIFY )
       {
        if ( event->mask & IN_ISDIR ) {
          printf( "New directory %s created.\n", event->name );
        }
        else {
            
          printf( "New file %s created.\n", event->name );
             auto input_data = Aws::MakeShared<Aws::FStream>("PutObjectInputStream",
   "detections/litter.bmp", std::ios_base::in | std::ios_base::binary);

       
          object_request.SetBody(input_data);

auto put_object_outcome = s3_client.PutObject(object_request);

if (put_object_outcome.IsSuccess())
{
    std::cout << "Done!" << std::endl;
}
else
{
    std::cout << "PutObject error: " <<
        put_object_outcome.GetError().GetExceptionName() << " " <<
        put_object_outcome.GetError().GetMessage() << std::endl;
}	
          
        }
      }
    
    }
    i += EVENT_SIZE + event->len;
  }
}
 
 
	 
	 
 
 return 0;
 }
