# Ender launcher: Software der Drohne per Befehl von der Funke starten

Ein kleiner Dienst, der auf dem **Host** läuft (nicht im ROS-Container) und auf feste Befehle hört. Er führt die
Schritte aus, die man sonst per SSH im Terminal tippt, und meldet jeden Schritt zurück. Man kann alles auf einmal
starten oder **jeden Schritt einzeln nacheinander**, um beim Fehlersuchen genau zu sehen, wo es hakt. Die App bekommt
Buttons dafür (Etappe 2).

Stand: **Etappe 1** (Dienst, Sim-Profil, Test-Client, Tests) und **Etappe 2** (Button "Prozesse" mit Liste in der
Punktwolken-Ansicht der App, `src/PointCloud/LauncherClient.*`). Das Profil für die echte Drohne (`profiles/real.json`,
dazu `systemd/ender-launcher.service`) ist geschrieben, aber noch nicht auf der Drohne getestet.

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
- `stop_step <Nr>` beendet nur diesen Schritt (im Test-Client: `stop-planer` und `stop-nodes`), wenn das Profil dafür einen eigenen Befehl (`stop`) kennt. Die anderen
  Schritte bleiben, wie sie sind. Im Sim-Profil geht das für den Planer (SUPER) und die Ender-Nodes, Container und Simulation beendest du mit dem
  allgemeinen `stop`.
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
python3 ender/launcher/launcher_client.py 127.0.0.1 stop-nodes            # nur die Nodes beenden, dann neu starten
python3 ender/launcher/launcher_client.py 127.0.0.1 stop-planer            # nur den Planer (SUPER) beenden
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
2. Etappe 2 in der Sim auf dem Laptop und der Funke prüfen. Für die echte Drohne: "Stoppen" im Flug sperren, solange die
   Drohne scharf ist.
3. Profil `real` auf dem Onboard-PC prüfen (Container, Flight Controller, Zenoh-Router, `sr` mit den Ender-Nodes im
   Block von `start_real.sh`, SUPER) und den Dienst per systemd beim Einschalten starten (`systemd/ender-launcher.service`).
   Beachten: `sr` ruft `cleanup.sh` auf (beendet tmux und `ros2`-Prozesse). Der `stop_guard` sperrt nur `stop` und
   `stop_step`, nicht das erneute Starten (`start`, `step`) der Wahrnehmung, das über `cleanup.sh` im Flug ebenfalls
   alles beendet. Der Dienst lauscht standardmäßig auf allen Schnittstellen (`--bind`, `--token` sind vorhanden).

## Protokoll ELCH v1 (UDP, JSON, Port 19903)

Anfrage: `{"v":1,"seq":7,"cmd":"start"|"stop"|"status"|"step"|"stop_step","step":2,"token":"..."}`
(`step` = Nummer des Schritts, nur bei `step` und `stop_step`, null-basiert; `token` nur, wenn der Dienst eines verlangt).

Antwort (gleiche `seq`, immer unter 1200 Byte):
`{"v":1,"seq":7,"profile":"sim","state":"partial","message":"","steps":[{"name":"Container","state":"ok","detail":"","can_stop":false}]}`

`state`: `stopped` (kein Schritt fertig), `partial` (einige fertig), `running` (alle fertig), `starting` und `stopping`
(der Dienst arbeitet), `error` (ein Schritt ist fehlgeschlagen). Schritt-`state`: `pending`, `running`, `ok`, `failed`.
`can_stop`: der Schritt lässt sich einzeln stoppen. Eine unbekannte Schrittnummer wird mit `{"error":"unknown step"}`
beantwortet. Pakete mit falscher Version, ohne `seq` oder mit falschem Token werden ohne Antwort verworfen.
