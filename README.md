#Autoelektronika-projekat
##Uvod
Zadatak ovog projekta je da simulira sistem za merenje temperature u automobilu, u radnom okruženju VisualStudio2019. Projekat je urađen po MISRA standardu.
##Zahtevi projekta:
1.Trenutne vrednosti temperature potrebno je dobiti sa kanala 0 serijske komunikacije. Primati odvojeno po 1 podatak za unutrašnju temperaturu i 1 podatak za spoljašnju temperaturu. Temperatura se dobija kao vrednost otpornosti u opsegu od 0-32 oma. 
2. Pratiti vrednosti obe temperature, uzimati poslednjih 5 očitavanja i usrednjavati ih.
3. Sa kanala 1 serijske komunikacije potrebno je poslati minimalne i maksimalne vrednosti za temperature i otpornosti, odnosno neophodno je kalibrisati vrednosti otpornosti za obe temperature. Preračunate vrednosti koje se dobijaju iz vrednosti otpornosti (izmedju MINTEMP I MAXTEMP) predstavljaju temperature u stepenima celzijusa.
4. Sa kanala 2 serijske komunikacije potrebno je poslati granične temperature THIGH i TLOW, na osnovu kojih stiže obaveštenje da temperatura nije u željenom opsegu.
5. Ako temperatura nije u željenom opsegu neophodno je da jedan stubac LED bara zasvetli tri puta periodom od 500ms.
6. Na LCD displeju prikazati izmerene podatke, brzina osvežavanja podataka treba da je 100ms. Ako je pritisnut prvi taster od dole u trećoj koloni na LED baru, na LCD dispelju treba da bude prikazana unutrašnja temperatura i otpornost, a ako je pritisnut drugi taster od dole u trećoj koloni- spoljašnja temperatura i otpornost.
##Periferije
Korišćene periferije su:
1. AdvUniCom - simulira serijsku komunikaciju. Iz Command Prompt-a pokreću se sva tri kanala, pojedinačnim upisom AdvUniCom 0, AdvUniCom 1 i AdvUniCom 2.
2. LED_bars_plus - periferija koja simulira diode i tastere. Iz Command Prompt-a pokreće se upisom LED_bars_plus RGb , prve dve kolone LED bara predstavljaju diode, a treća tastere.
3. Seg7_mux - predstavlja LCD displej. Iz Command Prompt-a pokreće se upisom Seg7_mux 7, kako bi se dobio displej sa 7 cifara.
##Simulacija sistema
Pre pokretanja programa neophodno je uključiti sve periferije po prethodno definisanim uputstvima. 
Nakon pokretanja programa u polje T1 kanala 0 AdvUniCom softvera upisati "u", kako bi se automaski slala vrednost unete otpornosti nakon pristiglog karaktera "u". U polje R1 uneti željenu otpornost u formatu UXXF (za unutrašnju) ili SXXF (za spoljašnju), gde XX predstavlja unetu otpornost, S ili U predstavljaju početak poruke, a F kraj poruke. Pritisnuti ok1 zatim Auto1 prilikom prvog slanja vrednosti, dok je prilikom ažuriranja vrednosti dovoljno pritisnuti samo ok1. Prilikom slanja jednocifrenih vrednosti otpornosti potrebno je upisati u formatu 0X.
U polje T1 kanala 2 AdvUniCom softvera upisati "t", kako bi se automaski slale vrednosti željenog opsega temperature, unete  nakon pristiglog karaktera "t". U polje R1 uneti željenu temperaturu u formatu TXXXXF, gde prva dva XX predstavljaju maksimalnu dozvoljenu temperaturu, a druga dva XX minimalnu dozvoljenu temperaturu. T predstavlja početak poruke, a F kraj poruke. Pritisnuti ok1 zatim Auto1 prilikom prvog slanja vrednosti, dok je prilikom ažuriranja vrednosti dovoljno pritisnuti samo ok1. Prilikom slanja jednocifrenih vrednosti otpornosti potrebno je upisati u formatu 0X.
Sa kanala 1 serijske komunikacije šalju se vrednosti temperature i otpornosti. U formatu \00MINTEMPXXXXCR prva dva XX predstavljaju minimalnu temperaturu, a druga dva XX maksimalnu otpornost. U formatu \00MAXTEMPXXXXCR prva dva XX predstavljaju maksimalnu temperaturu, a druga dva XX minimalnu otpornost. Poruka se šalje pritiskom na polje SEND CODE. Takođe za jednocifrene brojeve potrebno je pisati u formatu 0X.  
Na osnovu ovih vrednosti vrši se kalibracija i dobija se temperatura u stepenima celzijusa, čija vrednost se ispisuje na terminalu. Nakon kalibracije vrši se provera da li se temperatura nalazi u opsegu definisanom preko kanala 2. Ako unutrašnja temperatura nije u željenom opsegu 3 puta će zasvetleli prvi stubac LED bara, a ako spoljašnja temperatura nije u opsegu zasvetleće drugi stubac.
Za ispis vrednosti temperature i otpornosti na LCD displej potrebno je pritisnuti prvi taster od dole treće kolone za unutrašnju temperaturu i otpornost ili drugi taster od dole treće kolone za ispis spoljašnje.
##Opis taskova 
SerialSend_Task0 - omogućava automatsko slanje vrednosti otpornosti, tako što program svakih 1000ms šalje karakter "u" ka kanalu 0 i vrednost otpornosti se automatski ažurira.
SerialReceive_Task0 - task koji prihvata vrednosti otpornosti sa kanala 0 i šalje u redove kako bi se kasnije ovi podaci koristili u drugim taskovima.
SerialReceiveTask_1 - task koji prihvata vrednosti minimalne i maksimalne temperature i otpornosti sa kanala 1, potrebnih za kalibraciju. Ovi podaci se takođe šalju u red kako bi se omogućila komunikacija sa drugim taskovima.
SerialSend_Task2 - omogućava automatsko slanje vrednosti granične temperature, tako što program svakih 1000ms šalje karakter "t" ka kanalu 2 i vrednosti THIGH i TLOW se automatski ažuriraju.
SerialReceiveTask_2 - task koji prihvata vrednosti granične temperature, smešta ih u promenljive thigh i tlow, nakon čega ih šalje u red.
prosecna_temp_un - task za izračunavanje prosečne unutrašnje temperature na osnovu 5 uzastopnih merenja.
prosecna_temp_sp - task za izračunavanje prosečne spoljašnje temperature na osnovu 5 uzastopnih merenja.
kalibracija - task koji vrši kalibraciju temperature na osnovu otpornosti dobijene sa kanala 0, i minimalnih i maksimalnih vrednosti temperature i otpornosti dobijenih sa kanala 1. U tasku je implementirana signalizacija ukoliko se kalibrisana temperatura ne nalazi u željenom opsegu, tako što se uključuje odgovarajući stubac led bara.
timer_seg7 - tajmerska funkcija koja se poziva svakih 100ms i proverava stanje tastera na led baru. Informacija sa led bara daje odgovarajući semafor kako bi se ta informacija kasnije obradila.
mux_seg7_un - task pomoću kojeg se ispisuju vrednosti unutrašnje temperature i otpornosti na seg7_mux, kada preuzme odgovarajući semafor.
mux_seg7_sp - task pomocu kojeg se ispisuju vrednosti spoljasnje temperature i otpornosti na seg7_mux, kada preuzme odgovarajući semafor.
main_demo - predstavlja funkciju u kojoj se vrši inicijalizacija svih periferija koje se koriste, kreiraju se taskovi, semafori, redovi i tajmer. Takođe se definiše interrupt za serijsku komunikaciju i poziva vTaskStartScheduler() funkcija potrebna za raspoređivanje taskova.
