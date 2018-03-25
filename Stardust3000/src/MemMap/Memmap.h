/*
 * Memmap.h
 *
 *  Created on: 16 mars 2018
 *      Author: maxip
 */
	/*actionneurs */
		/*moteurs*/
			/*Colonnes*/
			struct Dcolonne { //colonne
				const uint8_t MCC1_1 = 0x11;
				const uint8_t MCC1_2 = 0x12;
				const uint8_t MCC2_1 = 0x13;
				const uint8_t MCC2_2 = 0x14;
			};
			/*deplacement*/
			struct Ddeplacement {
				const uint8_t MD_avant = 0x00;
				const uint8_t MD_droite = 0x01;
				const uint8_t MD_gauche = 0x02;
			};
		struct Dmoteur {
				Dcolonne colonne;
				Ddeplacement deplacement;
		};
	/*Capteurs*/
		/*couleur*/
		struct Dcouleur {
			const uint8_t CC_Cube = 0x80;
		};
		/*obstacle*/
		struct Dobstacle {
			const uint8_t CO_avant =0x81;
			const uint8_t CO_droite =0x82;
			const uint8_t CO_gauche =0x82;
		};
	struct Dcapteur {
		Dcouleur couleurs;
		Dobstacle obstacles;
	};
struct addrDevDescription {
		Dmoteur moteur;
		Dcapteur capteur;
}addrMap;

