#include "FlightTaskBurkut.hpp"

#include <cmath>

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/Publication.hpp>

using namespace matrix;

vehicle_land_detected_s _land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};
uORB::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};

bool FlightTaskBurkut::activate(vehicle_local_position_setpoint_s last_setpoint)
{

	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_position_setpoint(2) = _position(2);
	_yaw_setpoint = _yaw;


	// Pozisyonlar hafızaya alınır.
	_origin_z = _position(2);// 2 Metre _origin_z ile tutulur.
	_origin_y = _position(1);// x,y = 0,0 konumları + yaw açısı saklanır.
	_origin_x = _position(0);
	_origin_yaw = _yaw;

	return ret;

}


bool FlightTaskBurkut::update()
{

	// Update önce stage durumunu kontrol edicek.Önce stage-0 ile başlayacak.
	//stage-0 drone 2 metre yüksekte olduğu konum.
	//Burkut 2 metreye manuel yada qgc tarafından çıakrılıcak sonrasında parametre aktif edilecek.

	//_stage = 0 by default.

	switch (_stage)
	{
	case 0:

		//5 metre ileri giderken dönme işlemi başlar.

		_counter = _counter + _counter_speed; // adımı büyütüyoruz
		_position_setpoint(0) = _origin_x + _counter; // büyütülen adımı güncelliyoruz.

		_yaw_setpoint = _yaw_speed * _counter * 3.141592653589793f / 180.f; //dönüş hareketi yaptırıyoruz.

		//görev aşaması başarı ile tamamlandı ise
		//stage değişimi öncesi hazırlık ve stage değişimi yapılır.
		if(  _counter  >= 5.0f   )
		{
			_yaw_setpoint = _origin_yaw;// yönümüzü orjinal yöne eşitleriz.
			_stage = 1; //stage-1 e geçilir.
			//stage değiştiği için counter sıfırlanır ki sonraki görevde tekrar kullanılabilsin.
			_counter = 0.0f;//counter sıfırlanır.


			//sonra hepsi sonraki görev için hafızaya alınır.
			_origin_x = _origin_x + 5.0f;// x ekseni 5 metre taşınır.Diğer eksenlerde oynama yok.
			//bu sebeple diğerleri değiştirilmez.


			// ***********  alınan origin => x,y,z = 5,0,2 / yaw = 0 **********


		}


		break;
	case 1:
		_yaw_setpoint = _origin_yaw;// yönümüzü orjinal yöne eşitleriz.

		// 5x5 hipotenüs geçilecek.
		// 5 metre geri ve 5 metre yukarı adımlarından oluşacak bir stage.
		//counter 0 dan başlayarak 5 e kadar artıcak iki eksen içinde negatif olucak şekilde
		//setpointe eklencek.Bu sayede smooth bir hareket sağlanacak.
		//bir önceki stage de counter sıfırlanmıştı.Sorunsuz şekilde devam edebiliriz.
		_counter = _counter + _counter_speed; // adımı büyütüyoruz

		_position_setpoint(0) = _origin_x - _counter; // 5 metre geri
		_position_setpoint(1) = _origin_y; //y ekseni her zaman aynı origin_y değerine eşit.Ve bu değer değişmemeli.
		_position_setpoint(2) = _origin_z - _counter; // 5 metre yukarı

		//5 metre olunca ve stage-2 geçeceğiz.


		if( _counter >= 5.0f )
		{

			//sonra hepsi sonraki görev için hafızaya alınır.
			_origin_z = _origin_z - 5.0f; //-2 metreyi -5 ekleyerek -7 yaptık (NED FRAME)
			_origin_x = _origin_x - 5.0f; // 5 metreden 5 çıkarak 0 yaptık.
			//y ekseni her zamanki gibi sabit.
			//counter sıfırlanmalı sonraki görev için.
			_counter = 0.0f;
			// ************* alınan origin => x,y,z = 0,0,7 ***************
			_stage = 2;

		}

		break;
	case 2:
		_counter = _counter + _counter_speed; // z ekseninde 5 metre aşağı inme adımları
		_position_setpoint(2) = _origin_z + _counter; // adımları + olarak ekliyoruz çünkü şuan değer -7 , -7+5 = -2 (hedef 2m NED FRAME)
		_yaw_setpoint = _yaw_speed * _counter * 3.141592653589793f / 180.f; //dönüş hareketi yaptırıyoruz.

		if(  _counter  >= 5.0f   )
		{

			//sonra hepsi sonraki görev için hafızaya alınır.
			_origin_z = _origin_z + 5.0f;
			//y ekseni korunur
			//x eksen, korunur

			// ************* alınan origin => x,y,z = 0,0,2 ***************
			_counter = 0.0f; // counterın sıfırlandığını kontrol edip sonraki stage i öyle başlatıyoruz.
			_stage = 3; // daire çizmeye başla , stage-3 geçilsin.
			//sonraki görev için gerekli parametreler.Loop değil activate gibi
			//çalışması için burdaki geçiş koşulunun içerisine yazılır.
			//her loop başlangıcında bir değeri tekrar atamak zorunda kalmasın diye.
			_counter_speed = 0.1f;//artık derece olduğu için 10 kat hızlandırıyoruz.
			_yaw_speed = 30.0f; //yaw hızıda buna bağlı yavaşlatılır.
		}

		break;
	case 3:
		// alttaki 2 satır bir önceki stage de yazılıcağından burda commentout edildi.
		//_counter_speed = 0.1f;//artık derece olduğu için 10 kat hızlandırıyoruz.
		//_yaw_speed = 30.0f; //yaw hızıda buna bağlı yavaşlatılır.

		_counter = _counter + _counter_speed;

		//convertin degree to radian
		_radian_of_degree = _counter * 3.141592653589793f / 180.f;

		//setting new setpoints
		_position_setpoint(0) = _origin_x + (_radius_of_chamber * sinf(_radian_of_degree));
		_position_setpoint(2) = _origin_z - 3.0f + (_radius_of_chamber * cosf(_radian_of_degree));
		_yaw_setpoint = _yaw_speed * _counter * 3.141592653589793f / 180.f;

		if(_counter >= 360.0f)
		{
			//görev sonu origin değişmeyeceği için hiçbirinin tekrar atanmasına gerek yok.
			// ************* alınan origin => x,y,z = 0,0,2 ***************
			_counter = 0.0f;
			_stage = 4;
		}

		break;
	case 4://iniş

		_position_setpoint = Vector3f(_origin_x,_origin_y,NAN);//konum sabitle
		_velocity_setpoint(2) = 0.2f; // aşağı hız

		//disarm zamanı 160 yada 1.98 metre aşağısı

		if(  ( _counter >= 160.0f) || (_position(2)-_origin_z > 1.98f) )
		{
			_vehicle_land_detected_pub.publish(_land_detected);
			_stage = 5;
		}


		break;
	case 5://bitiş
		_vehicle_land_detected_pub.publish(_land_detected);
		break;

	default:
		break;
	}







	return true;
}
