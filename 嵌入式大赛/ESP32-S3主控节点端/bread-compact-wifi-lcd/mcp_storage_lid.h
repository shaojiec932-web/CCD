#ifndef _MCP_STORAGE_LID_H_
#define _MCP_STORAGE_LID_H_

void RegisterMcpStorageLid();
void StorageLidInit();
void StorageLidOpen(bool auto_close);
void StorageLidClose(bool notify);
bool StorageLidIsOpen();
const char* StorageLidStateText();

#endif // _MCP_STORAGE_LID_H_
