#include <labust/tritech/DiverMsg.hpp>

int main(int argc, char* argv[])
{
	using namespace labust::tritech;
	Bits2LatLon bits;
	double initLat = 43.5101;
	double initLon = 16.3909;
	bits.setInitLatLon(initLat, initLon);

	LatLon2Bits bitsout;

	bitsout.convert(43.510306618, 16.3908828623, 22);

	unsigned char* pt1 = reinterpret_cast<unsigned char*>(&bitsout.lat);
	unsigned char* pt2 = reinterpret_cast<unsigned char*>(&bitsout.lon);

	for(int i=0; i<4; ++i)
	{
		std::cout<<std::hex<<int(pt1[i])<<std::endl;
	}

	std::cout<<std::endl;

	for(int i=0; i<4; ++i)
	{
		std::cout<<std::hex<<int(pt2[i])<<std::endl;
	}

	std::cout<<"Lat:"<<bitsout.lat<<","<<bitsout.lon<<std::endl;
	bits.convert(bitsout.lat, bitsout.lon, 14);

	std::cout.precision(12);
	std::cout<<"Latitude:"<<bits.latitude<<","<<bits.longitude;

	return 0;
}
