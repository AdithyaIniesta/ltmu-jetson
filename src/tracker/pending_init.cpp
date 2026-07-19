#include "pending_init.h"

std::mutex g_pendingInitMtx;
bool g_pendingInit = false;
cv::Rect g_pendingInitBbox;
