#ifndef _BEACON_HEADER_
#define _BEACON_HEADER_

/*
Pour les types de modèles, les groupes sont les suivants :
 - Groupe 1 : aérostat captif / aéromodèle de vol circulaire / aéromodèle de vol libre / montgolfière
 - Groupe 2 : planeur, aile (non motorisé) / dirigeable / parachute, parapente / aéronef à ailes battantes
 - Groupe 3 : hélicoptère / multirotors / convertible / combiné / paramoteur / autogire
 - Groupe 4 : avion, aile, planeur (motorisé)
*/
typedef enum {
	MODEL_GROUP1 = 0,
	MODEL_GROUP2 = 1,
	MODEL_GROUP3 = 2,
	MODEL_GROUP4 = 3,
} model_group;

/*
Pour les plages de masse, les groupes sont les suivants :
 - Entre 800 g et 2 kg (model_mass = 0)
 - Entre 2 kg et 4 kg (model_mass = 1)
 - Entre 4 kg et 25 kg (model_mass = 2)
 - Entre 25 kg et 150 kg (model_mass = 3)
 - Plus de 150 kg (not supported)
*/
typedef enum {
	MODEL_MASS_800GR_2KG = 0,
	MODEL_MASS_2KG_4KG = 1,
	MODEL_MASS_4KG_25KG = 2,
	MODEL_MASS_25KG_150KG = 3,
} model_mass;

typedef struct {
	char builder_id[4];
	char version_id[4];
	char mac[13];
	model_group group;
	model_mass mass;
	char mass_str[4];
} beacon_data;

int beacon_set_home(double lat, double lng, double alt);
bool beacon_is_home_set();
int beacon_update_data(double latitude, double longitude, double altitude, double course, double speed);
bool beacon_data_must_be_send();
int beacon_send_data();
int beacon_init();

#endif /* _BEACON_HEADER_ */
