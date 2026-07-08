#ifndef _MCP_UWB_NAVIGATION_H_
#define _MCP_UWB_NAVIGATION_H_

#include <string>

void RegisterMcpUwbNavigation();
bool UwbNavigationStartPositioning();
void UwbNavigationStop();
std::string UwbNavigationGetTelemetryJson();

#endif // _MCP_UWB_NAVIGATION_H_