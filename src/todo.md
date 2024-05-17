Co mam:
* tworzenie ścieżki poprzez klikanie mapy;  tworzenie poprzez ręczne wpisanie punktów z mapy
* ogarnięta transformacja pomiędzy map -> odom, czyli mogę posługiwać się kordynatami stworzonej mapy, a to zostanie odczytane na spokojnie w frame lokalnym robota "odom",
* Stworzona kara za zbyt szybkie zmiany wychylenia kierownicy (macierz R)


Co trzeba:
* Zrobić testy: (parametr N, macierz R)
* zobaczyć jak działa jazda tyłem
* wymyślić sposób jak zebrać wyniki (tylko csv w mainMPC, czy może też odpalić rosbag, żeby zwiększyć częstotliwość i dokładność odom)
* zapytać się czy nie ma jakiejś bardziej pustej sali, żeby móc ustawić tor i porobić różne scenariusze
* scenariusz może że chwila jazda szybko, a później wolno i tak na zmianę;  Na pewno jakiś że robienie kółeczek;    pusta sala to całkiem okrąg by mozna było zrobić o zadanym promieniu; na pewno też jakiś łuk
* może ogarnąć w końcu zbieżność
* zastanowić się czy przy wynikach odnosić się do wyniku z odom_combined (bo to jest jednak wejście do naszego algorytmu), czy użyć jakiejś innej miary położenia


Do zastanowienia:
* zobaczyć jak działa TebLocalPlaner(albo coś innego) i dodać w MPC jeszcze śledzenie orientacji, a nie tylko położenia
* optymalizacja pod względem czasu, ale to chyba bez sensu