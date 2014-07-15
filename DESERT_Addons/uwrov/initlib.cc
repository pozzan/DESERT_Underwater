#include<tclcl.h>

extern EmbeddedTcl UwrovTclCode;

extern "C" int Uwrov_Init() {
	UwrovTclCode.load();
	return 0;
}
