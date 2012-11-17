#include "semantic_mapping_app/parameter_server.h"

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
    pre = ros::this_node::getName();
    pre += "/config/";

    defaultConfig();
    getValues();
}

ParameterServer* ParameterServer::instance() {
    if (_instance == NULL) {
        _instance = new ParameterServer();
    }
    return _instance;
}
void ParameterServer::defaultConfig() {

  config["mesh_input_filename"]         = std::string("/work/kidson/meshes/cabinet_scan_2/mesh_1.ply");
  //config["start_paused"]              = static_cast<bool> (true);
}

void ParameterServer::getValues() {
    map<string, boost::any>::const_iterator itr;
    for (itr = config.begin(); itr != config.end(); ++itr) {
        string name = itr->first;
        if (itr->second.type() == typeid(string)) {
            config[name] = getFromParameterServer<string> (pre + name,
                    boost::any_cast<string>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<string>(itr->second));
        } else if (itr->second.type() == typeid(int)) {
            config[name] = getFromParameterServer<int> (pre + name,
                    boost::any_cast<int>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
        } else if (itr->second.type() == typeid(double)) {
            config[name] = getFromParameterServer<double> (pre + name,
                    boost::any_cast<double>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
        } else if (itr->second.type() == typeid(bool)) {
            config[name] = getFromParameterServer<bool> (pre + name,
                    boost::any_cast<bool>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
        }
    }
}

