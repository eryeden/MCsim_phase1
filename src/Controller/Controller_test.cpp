#include <Controller/Controller_test.hpp>

using namespace Controller;

Controller_test::Controller_test(MC::Core & _mc_core) 
	:Base(_mc_core)
{
	;
}

Controller_test::~Controller_test() {
	
}

void Controller_test::Initialize() {
	Set_w_m_All(0);
}

void Controller_test::Update() {
	Set_w_m_All(0);
}

void Controller_test::Set_w_m_All(float _wm) {
	//for (auto itr = core.mtrplps.begin(); itr != core.mtrplps.end(); itr++) {
	//	(*itr)->w_m = _wm;
	//}

	//‚±‚ê‚ÅFor-Each•—‚ÉŽg‚¦‚é‚ç‚µ‚¢
	for (auto itr : core.mtrplps) {
		itr->w_m = 0;
	}



}



