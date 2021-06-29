#include <auto_uav/server.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "server_node");
    Server::server server;
    server.run();
    return 0;
}