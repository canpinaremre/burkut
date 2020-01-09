#include "FlightTaskBurkut.hpp"


using namespace matrix;

/*
KULLANISLI FONKSIYONLAR

uORB::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};
uORB::Publication<position_setpoint_s>	_position_setpoint_pub{ORB_ID(position_setpoint)};

*/
int sonuc = 3;
void FlightTaskBurkut::_publishVehicleCmdDoLand()
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LAND;

	// publish the vehicle command
	_pub_vehicle_command.publish(command);

	_param_mpc_auto_mode.set(_default_mpc_auto_mode);
	_param_mpc_auto_mode.commit();
	updateParams();

}


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
			_position_setpoint = Vector3f(NAN,NAN,NAN);
			_velocity_setpoint = Vector3f(NAN,NAN,NAN);

		}

		break;
	case 4://iniş


		// publish the vehicle command
		_publishVehicleCmdDoLand();



		//In ve komut vermeyi kes
		_stage = 5;

		break;
	case 5:
		//Bitis
		break;

	default:
		break;
	}







	return true;
}
