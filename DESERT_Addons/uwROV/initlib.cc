#include<tclcl.h>

extern EmbeddedTcl UwROVTclCode;

extern "C" int Uwrov_Init() {
	UwROVTclCode.load();
	return 0;
}
