#ifndef _MCP_RFID_CARGO_MANAGER_H_
#define _MCP_RFID_CARGO_MANAGER_H_

#include <string>

void RegisterMcpRfidCargoManager();
std::string RfidCargoStartCollecting();
std::string RfidCargoStartCollecting(const std::string& target_name);
std::string RfidCargoScanOnce();
std::string RfidCargoFinishCollecting();
std::string RfidCargoClear();
std::string RfidCargoGetStatusJson();

#endif // _MCP_RFID_CARGO_MANAGER_H_
