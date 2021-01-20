%% ASE-2411 JSH Harjoitustyö, kevät 2020
% Takaisinkytektyn säädön suunnittelu Quanser QUBE-Servo 2 -järjestelmälle.
%
% Tekijä: Lauri Aaltonen
% Opnum: 267118
% Aihe: 3. Kulmanopeuden säätö, PID
% 
%% Alkutiedot

% DC-moottorin parametrit:
Rm = 8.4;
kt = 0.042;
km = 0.042;
Jm = 4*10^(-6);
Jh = 0.6*10^(-6);
% Kuormana oleva py�riv� kiekko:
md = 0.053;
rd = 0.0248;
% Kiekon hitausmomentti Jd:
Jd = 0.5*md*rd^2;
% Kokonaishitausmomentti Jeq:
Jeq = Jm + Jh + Jd;
%% Suunnitellaan säädin napojen asettelumenetelmällä (PI-säädin)

s = tf('s');
G_prosessi = kt / (Rm*Jeq*s+km*kt); % prosessi on 1.kl viiveetön

% kumotaan prosessin hitain napa:
% 0.0001755 s + 0.001764 = 0 prosessin "alakerta"
% -> napa = -0.001764/0.0001755
% --> 1/Ti = 0.001764/0.0001755
% Ti = 1/(0.001764/0.0001755);
Ti = 1/((km*kt)/(Rm*Jeq));

% figure; bode(G_prosessi*(s+1/Ti)/s) % Kp=1 

Kp = 1.2*10^(-27.5/20);  % 10 rad/s kohdasta desibeli-kuvaajasta

% eli 
G_pi = Kp*(s+1/Ti)/s;  % lopullinen suunniteltu säädin



% 141_erosuure * Kp = 10 V alussa 
% (Huom asetusarvosuotimen kanssa erosuure noin pieni)
% --> Kp = 0.07
% ohjausta kuitenkin vähän liikaa
% --> Kp = 0.05 on parempi

%% Suotimet

% Kuten kerrottu: mittausta vain kiekon asennosta.
% --> rakensin 1/s-lohkon ja sen kanssa sarjaan alipäästösuotimen
% Alipäästösuotimen tau = 0.1 antaa hyvän mittauksen, liki sama kuin
% ideaalinen kulmanopeus (josta ei mittausta mutta nähdään simulaattorista)
% Jos tau:ta kasvattaa niin mittaus seuraa "laiskemmin" todellista arvoa
% Jos kulmanopeus tekee äkkinäisiä muutoksia kuten teräviä kulmia niin mitä
% suurempi tau niin sitä tylpempi kulma mittauskanavassa.
% 
% Työssä siis takaisinkytkennässä käytetään tätä alipääästösuotimen ja
% derivaattorin sarjankytkentää kulma-tiedosta
% erosuureen laskemisessa ja TODELLISTA kulmanopeutta PO, asettumisaika
% yms laskuihin
M = s/(0.1*s+1);

% Lisäksi rakensin alipäästösuotimen myös asetusarvosuotimeksi (tau = 0.5)
F = 1/(0.2*s+1); % mittaushaaran alipäästö ja derivaattori
% Suotimen lisäämisen jälkeen ohjaussignaali pysyy 
% alle 10 V (monotoninen muoto jos tau olisi 0.5)

% Jos takaisinkytkennän suotimen tau > asetusarvosuotimen tau
% ---> ohjaussignaaliin kauhea piikki (plus värähtelyä) 
% kun asetusarvo muuttuu

%% Verifioi/Varmista, että vaatimusm��rittelyt toteutuvat.
% Aja simulaattori tätä kohtaa ennen että saadaan workspaceen tiedot!!!
open('testi_sim.slx')
sim('testi_sim.slx')


% • Kulmanopeuden askelsarjatestisignaali: ± 200 rad/s.
% asetettu simulaattoriin (signal generator)
figure; plot(asetusarvo.time, asetusarvo.signals.values)
xlabel('aika t (sekunti)'), ylabel('Askelvaste \omega (rad/s)')
title('Askelsarjatestisignaali')


% •Askelvasteen asettumisaika Ts mahdollisimman lyhyt. Tässä asettumisaika mitataan siitä
%   hetkestä t, kun ulostulo pysyy ±5 % päässä asetusarvostaan. 
% • Askelvasteen prosentuaalinen ylitys ≤ 5 %.
figure; plot(omega.time, omega.signals.values)
xlabel('aika t (sekunti)'), ylabel('todellinen kulmanopeus \omega (rad/s)')
title('Todellinen ja mitattu kulmanopeus')
hold on
plot(mittaus.time, mittaus.signals.values)
plot(asetusarvo.time, asetusarvo.signals.values)
legend('todellinen','mitattu','asetusarvosignaali')
hold off

% Kuvaajasta nähdään että ylitystä ei ole (PO=0%) ---> OK

% -200 rad/s --> 200 rad/s (delta=400, ±5% = ±20 rad/s putken sisään T_s) 
% Kuvaajasta nähdään että:
% T_s = noin 8.1764_(asetus-5%)-7.845s_(kun asetusarvo nousee ylös)
% T_s = 0.33 sekuntia about, alle sekunti niin omasta mielestä ihan okei
% (vaatimusmäärittelyssä ei T_s kiinnitetty, vain mahdollisimman lyhyt)
% Tietenkin riippuu tilanteesta, mikä on hyväksyttävä T_settling_time


% • Vaihevara ≥ 35 astetta, vahvistusvara ≥ 2 (6dB).
L = G_prosessi*G_pi*(1/s)*M;%(1/s)koska G_prosessi vain kulmanopeuteen asti
% ja M:ssä derivaattori ja alipäästö.
% "mittaus" kuuluu tavallaan prosessiin
% ja oikea mittaus ideaalinen = 1
[Gm,Pm] = margin(minreal(L))
% --> PM = 48 deg --> >35 deg --> OK
% --> GM = Inf --> >6dB --> OK
% ulostulo seuraa sisäänmenoa vaikka sisäänmenosignaali olisi 2 milj. rad/s
% jos kokeilee eri inputteja

% • Stabiiliusvara ≥ 0.5.
% s_m = 1/M_s = 1/1.53 = 0.65 --> >0.5 ---> OK
% M_s saatiin alla olevasta herkkyyden amplitudivaste-kuvaajasta (maksimi)
% (0.65 nähdään myös L_openloop:n ja NB:n etäisyyskuvaajasta, minimikohta)


% • Ohjausjännite pitää pysyä välillä: ±10V.
figure; plot(ohjaus.time, ohjaus.signals.values)
xlabel('aika t (sekunti)'), ylabel('ohjausjännite U (V)')
title('Ohjaussignaali')
% Kuvaajasta havaitaan että ohjausjännite on ±9.85 äärimmillään ---> OK
% Jos halutaan monotoninen ohjausjännite niin tau_asetusarvosuod
% suuremmaksi
% Asettelin Kp:n avulla ohjauksen mahdollisimman lähelle sallittua

%% Gang of six (sillä käytössä myös 2DOF-rakenne)
% Muista ajaa ennen tätä edellä ollut suotimet-kohta

% PCF/(1+PC)
% -PC/(1+PC)
% P/(1+PC)
% CF/(1+PC)
% -C/(1+PC)
% 1/(1+PC)

% mittaus otetaan tavallaan prosessin todel.ulostulo->integ.->ylipäästö
% (eli siis yhdistin mittauslohkon ja Qube-blokin lohkon, nopeudesta
% kulmaan. Mittauslohkosta osa on tavallaan ideaalisesti qubessa jo)
M = M/s;

% plottausasetukset, abs dB:n sijaan y-akselille
opts = bodeoptions;
opts.PhaseVisible = 'off';
opts.MagUnits = 'abs';

% Siirretään mittaus(s+alipäästö) asetusarvosuotimeen ja säätimeen
F_uusi = F/M;
C = G_pi*M;

% Sitten plottaukset...

figure;bodeplot(G_prosessi*C*F_uusi/(1+G_prosessi*C),opts);
title('PCF/(1+PC) amplitudivaste')
xlabel('\omega')
grid on

figure;bodeplot(-G_prosessi*C/(1+G_prosessi*C),opts);
title('-PC/(1+PC) amplitudivaste (complementary sensitivity)')
xlabel('\omega')
grid on

% Häiriöstä ulostuloon
figure;bodeplot(G_prosessi/(1+G_prosessi*C),opts);
title('P/(1+PC) amplitudivaste (load disturbance sensitivity)')
xlabel('\omega')
grid on

figure;bodeplot(F_uusi*C/(1+G_prosessi*C),opts);
title('CF/(1+PC) amplitudivaste')
xlabel('\omega')
grid on

% Mittauskohinasta ulostuloon
figure;bodeplot(-C/(1+G_prosessi*C),opts);
title('-C/(1+PC) amplitudivaste (noice sensitivity)')
xlabel('\omega')
grid on

figure;bodeplot(1/(1+G_prosessi*C),opts);
title('1/(1+PC) amplitudivaste (sensitivity)')
xlabel('\omega')
grid on
% Herkkyyden ampl. vahv. max on 1,534
% eli s = 1/1,534 = 0.65 --> >0.5 --> stabiiliusvara OK
% (saisi vaihtoehtoisesti myös (1+PC):n minimistä eli
% NB:n ja nyquist-openloop etäisyys)


