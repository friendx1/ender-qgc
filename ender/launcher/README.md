# Ender launcher: Software der Drohne per Befehl von der Funke starten

Ein kleiner Dienst, der auf dem **Host** läuft (nicht im ROS-Container) und auf feste Befehle hört. Er führt die
Schritte aus, die man sonst per SSH im Terminal tippt, und meldet jeden Schritt zurück. Man kann alles auf einmal
starten oder **jeden Schritt einzeln nacheinander**, um beim Fehlersuchen genau zu sehen, wo es hakt. Die App bekommt
später Buttons dafür (Etappe 2).

Stand: **Etappe 1** (Dienst, Sim-Profil, Test-Client, Tests). Die App kennt den Dienst noch nicht, das Profil für die
echte Drohne fehlt noch.

## Was er tut und was nicht

- Über das Netz kann man nur diese Befehle und eine Schrittnummer schicken, nie eine Kommandozeile. Was läuft, steht im Profil auf dem
  Rechner (`profiles/*.json`). Ein fremdes oder falsches Paket kann also nichts anderes auslösen.
- Optional ein gemeinsames Passwort (`--token`), das jede Anfrage tragen muss.
- Er startet nur die Wahrnehmung (Sim, Planer, Nodes für die Punktwolke). **Kein Arm, kein Takeoff.**
- `start` macht alle Schritte, die noch nicht fertig sind, der Reihe nach. Ist schon alles fertig oder arbeitet der
  Dienst gerade, passiert nichts doppelt (UDP-Pakete können doppelt ankommen, die App fragt bei Verlust einfach
  nochmal).
- `step <Nr>` führt nur diesen einen Schritt aus (auch wenn er schon fertig ist, dann startet er neu). So startest du die
  Prozesse einzeln nacheinander. Es wird nicht geprüft, ob die Schritte davor schon liefen, die Reihenfolge ist deine
  Sache.
- `stop_step <Nr>` beendet nur diesen Schritt, wenn das Profil dafür einen eigenen Befehl (`stop`) kennt. Die anderen
  Schritte bleiben, wie sie sind. Im Sim-Profil geht das für alle vier Schritte. Die Simulation merkt sich beim Start die PID ihrer
  Shell (`/tmp/ender_sim.pid`), `ender/sim/stop_tree.sh` beendet dann den ganzen Prozessbaum (erst SIGINT, dann SIGTERM,
  dann SIGKILL). Wer den Container stoppt (`stop_ends_all`), setzt damit auch alle Schritte danach zurück.
- Schritte behalten ihren Zustand (pending, running, ok, failed). Ein Häkchen heißt nur, dass der Schritt gestartet wurde
  und seine Bedingung zu dem Zeitpunkt erfüllt war, nicht, dass der Prozess heute noch lebt.
- `stop` bricht einen laufenden Start ab und führt dann die Stopp-Schritte aus. Mit `stop_guard` im Profil lässt sich
  das sperren (ein Befehl, der bei Erfolg `0` liefert, sonst wird "Stoppen gesperrt" gemeldet). Für die echte Drohne
  gehört dort hin: nicht im Flug.

## Sim ausprobieren (Linux-Rechner)

```bash
cd ~/ender-qgc
python3 ender/launcher/ender_launcher.py --profile sim          # Dienst, bleibt im Vordergrund
python3 ender/launcher/launcher_client.py 127.0.0.1 start --watch   # zweites Terminal: alles, was fehlt
python3 ender/launcher/launcher_client.py 127.0.0.1 status          # Schritte mit Nummern
python3 ender/launcher/launcher_client.py 127.0.0.1 stop --watch
```

**Einzeln nacheinander** (zum Fehlersuchen), Nummer wie in `status` oder ein Teil des Namens:
```bash
python3 ender/launcher/launcher_client.py 127.0.0.1 step 0 --watch        # Container
python3 ender/launcher/launcher_client.py 127.0.0.1 step Simulation --watch
python3 ender/launcher/launcher_client.py 127.0.0.1 step Planer --watch
python3 ender/launcher/launcher_client.py 127.0.0.1 step Nodes --watch
python3 ender/launcher/launcher_client.py 127.0.0.1 stop-step Nodes       # nur die Nodes beenden, dann neu starten
```
Schlägt ein Schritt fehl, steht der Fehler daneben. Beheben und denselben Schritt noch einmal mit `step` starten, die
anderen bleiben, wie sie sind.

Vom Laptop aus geht derselbe Client mit der IP des Linux-Rechners (UDP-Port **19903**, Firewall prüfen). Die Adresse,
von der der Startbefehl kommt, setzt der Dienst im Profil als `{client_ip}` ein: Dahin schickt die `bridge_node` die
Punktwolke. Wer `start` schickt, bekommt also den Scan.

Das Sim-Profil macht der Reihe nach: Container im Hintergrund starten (`start_sim_container.sh --detach`), `sl`,
`launch_super`, unsere Nodes (`start_ender_nodes.sh`, ohne sich an tmux zu hängen). Jeder Schritt wartet, bis sein
Topic da ist.

### Noch nicht gegen die echte Sim geprüft

Die Logik ist mit harmlosen Befehlen getestet (`python3 -m unittest discover -s ender/launcher`), die Befehle des
Sim-Profils nicht, denn `sl` und `launch_super` stammen aus Birks Container und sind hier nicht lesbar:

- Laufen `sl` und `launch_super` mit `docker exec -d ... zsh -ic '...'` ohne Terminal? Sie sind zsh-Funktionen. Hängen
  sie sich an tmux (`tmux attach` ohne Terminal schlägt fehl, die Sitzung bleibt aber bestehen), steht der Fehler im
  Schritt. Dann den Befehl im Profil anpassen.
- Die Wartebedingungen prüfen Topics (`cloud_registered_body`, `rog_map/inf_occ`). Stimmen die Namen in der Sim?
- Der Container hat jetzt zwei Betriebsarten: `start_sim_container.sh` (Terminal wie bisher) und `--detach`
  (Hintergrund). Läuft er schon mit Terminal, übernimmt der Dienst ihn einfach.

## Nächste Schritte

1. Sim: Schritte im Profil an die echten Befehle anpassen, bis `start` bis `running` durchläuft.
2. Etappe 2: Button und Statuszeile in der Punktwolken-Ansicht (UDP-Client in QGC, gleiche Datagramme wie der
   Test-Client). Im Flug sperren wir "Stoppen", solange die Drohne scharf ist.
3. Profil `real` für den Onboard-PC (Zenoh-Router, `sr`, Nodes, optional SUPER), Dienst per systemd beim Einschalten.
   Dafür brauchen wir Birks Skripte `start_real.sh`, `start_zenoh` und die Definition von `launch_super`.
   Beachten: `sr` ruft `cleanup.sh` auf (beendet tmux und `ros2`-Prozesse), unsere Nodes müssen danach starten.

## Protokoll ELCH v1 (UDP, JSON, Port 19903)

Anfrage: `{"v":1,"seq":7,"cmd":"start"|"stop"|"status"|"step"|"stop_step","step":2,"token":"..."}`
(`step` = Nummer des Schritts, nur bei `step` und `stop_step`, null-basiert; `token` nur, wenn der Dienst eines verlangt).

Antwort (gleiche `seq`, immer unter 1200 Byte):
`{"v":1,"seq":7,"profile":"sim","state":"partial","message":"","steps":[{"name":"Container","state":"ok","detail":"","can_stop":false}]}`

`state`: `stopped` (kein Schritt fertig), `partial` (einige fertig), `running` (alle fertig), `starting` und `stopping`
(der Dienst arbeitet), `error` (ein Schritt ist fehlgeschlagen). Schritt-`state`: `pending`, `running`, `ok`, `failed`.
`can_stop`: der Schritt lässt sich einzeln stoppen. Eine unbekannte Schrittnummer wird mit `{"error":"unknown step"}`
beantwortet. Pakete mit falscher Version, ohne `seq` oder mit falschem Token werden ohne Antwort verworfen.
