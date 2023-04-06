#include "talker.h"
zmq::context_t ctx;
zmq::socket_t skt(ctx, ZMQ_PUB);
zmq::socket_t subscriber(ctx, ZMQ_SUB);
Talker::Talker()
{
	converter_= make_unique<ImagemConverter>();
	sub_thread = std::thread([this] { Talker::Process(); });
}
Talker::~Talker() {
	while (sub_thread.joinable())
		sub_thread.join();
	skt.close();
	subscriber.close();
	ctx.close();
	converter_ = nullptr;
}

void Talker::Init(const std::string &ip) {
	skt.bind(ip);
	std::string TOPIC = "";
	subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length());
	subscriber.connect(ip);
}

void Talker::Pub(sonic_json::Document &metadata) {
	std::string json_str = metadata.Dump();
	zmq::const_buffer cb((json_str.c_str()), (json_str.size()));
	skt.send(cb);
}
// void Talker::Pub(cv::Mat &imput) {
// 	std::string string_data = converter_->mat2str(imput);
// 	nlohmann::json metadata = {
// 			{"Type",  4},
// 			{"Image",  string_data}};
// 	std::string json_str = metadata.dump();
// 	zmq::const_buffer cb((json_str.c_str()), (json_str.size()));
// 	skt.send(cb);
// }
void Talker::Process()
{
	while (subscriber.connected())
	{
		zmq::mutable_buffer message;
		if (subscriber.recv(message))
		{
			auto reply_str = std::string(static_cast<char *>(message.data()), message.size());
			sonic_json::Document doc;
			doc.Parse(reply_str);
			
			sonic_json::WriteBuffer wb;
			doc.Serialize(wb);
			std::cout << wb.ToString() << std::endl;
			
			// auto json_data = nlohmann::json::parse(reply_str);
			// // // 将图像消息进行转换
			// cv::Mat foo = converter.str2mat( json_data["data"]);
			// cv::imshow("rec", foo);
			// cv::waitKey(1);
		}
		std::chrono::milliseconds dura(1);
		std::this_thread::sleep_for(dura);
	}
}