## DDS_v1_7.gd
## ============
## Autoload (singleton globale) — Nodo DDS per la comunicazione UDP con il bridge.
##
## Implementa un protocollo DDS (Data Distribution Service) semplificato su UDP.
## Essendo registrato come Autoload in Project Settings, è istanziato automaticamente
## all'avvio di Godot e rimane in memoria per tutta la durata della sessione.
## È accessibile globalmente da qualsiasi script tramite il nome "DDS".
##
## Canale: UDP :4444 ← bridge_v1_7.py
##
## Variabili ricevute: X, Y, Theta, WheelLeft, WheelRight,
##                     ToF_L, ToF_CL, ToF_C, ToF_CR, ToF_R,
##                     TickId, T_robot_ms
##
## Uso:
##   DDS.subscribe("X")        # registra interesse per la variabile X
##   var x = DDS.read("X")    # legge il valore corrente di X
##   DDS.read("TickId")       # -1 = stale, >0 = dati validi

extends Node

# ── Costanti protocollo DDS ───────────────────────────────────────────────────

const COMMAND_KEEP_ALIVE = 0x80   # pacchetto heartbeat dal subscriber
const COMMAND_SUBSCRIBE  = 0x81   # richiesta di sottoscrizione a variabili
const COMMAND_PUBLISH    = 0x82   # pubblicazione di un valore da remoto

const DDS_TYPE_UNKNOWN = 0   # tipo variabile sconosciuto
const DDS_TYPE_INT     = 1   # variabile intera (int32)
const DDS_TYPE_FLOAT   = 2   # variabile float (float32)

const SERVER_PORT  = 4444    # porta UDP in ascolto (bridge → Godot)
const TTL_SECONDS  = 2.0     # tempo di vita subscriber senza keep-alive (s)

# ── Stato interno ─────────────────────────────────────────────────────────────

var udp_server  : UDPServer              # server UDP in ascolto su :4444
var udp_peers   : Array      = []        # lista peer connessi
var subscribers : Dictionary = {}        # peer → SubscribedVarCollection
var variables   : Dictionary = {}        # nome → DDSVariable (pubblicazione)
var _local_vars : Dictionary = {}        # nome → valore (lettura locale)

var current_tick_id  : int = 0    # TickId corrente ricevuto dal bridge
var last_receive_msec : int = 0   # timestamp msec dell'ultimo pacchetto valido
var last_valid_tick   : int = 0   # ultimo TickId valido ricevuto

# ── Classi interne ────────────────────────────────────────────────────────────

## DDSVariable — rappresenta una variabile DDS pubblicabile verso i subscriber.
## Mantiene il valore corrente, il pacchetto binario precompilato e la lista
## dei peer a cui pubblicare.
class DDSVariable:
	var name         : String          # nome della variabile
	var type         : int             # tipo: DDS_TYPE_INT o DDS_TYPE_FLOAT
	var value                          # valore corrente
	var peers        : Array           # lista PacketPeerUDP subscriber
	var packet       : PackedByteArray # pacchetto binario precompilato
	var value_offset : int             # offset nel pacchetto dove scrivere il valore

	## Inizializza la variabile con il nome specificato.
	func init(n: String) -> void:
		name         = n
		type         = 0
		value        = 0
		peers        = []
		value_offset = 0

	## Aggiunge un peer alla lista dei subscriber se non già presente.
	func add_peer(p: PacketPeerUDP) -> void:
		if p not in peers:
			peers.append(p)

	## Aggiorna il valore e riscrive il pacchetto.
	## Se il tipo cambia, ricompila l'header del pacchetto.
	func set_value(t: int, val) -> void:
		if t != type:
			type = t
			_prepare_packet()
		match type:
			1: packet.encode_s32(value_offset, int(val))
			2: packet.encode_float(value_offset, float(val))
		value = val

	## Precompila il pacchetto binario DDS per questa variabile.
	## Formato: [0x82][tipo][len_name][name...][valore]
	func _prepare_packet() -> void:
		packet = PackedByteArray()
		packet.resize(3 + len(name) + 4)
		packet.encode_u8(0, 0x82)       # command PUBLISH
		packet.encode_u8(1, type)        # tipo variabile
		packet.encode_u8(2, len(name))   # lunghezza nome
		var i = 3
		for c in name.to_ascii_buffer():
			packet.encode_u8(i, c)
			i += 1
		value_offset = i                 # offset dove scrivere il valore

	## Invia il pacchetto a tutti i peer subscriber.
	func publish() -> void:
		for p in peers:
			p.put_packet(packet)

## SubscribedVarCollection — gestisce un subscriber remoto con TTL.
## Ogni peer connesso ha la sua collezione di variabili sottoscritte.
## Se non invia keep-alive entro TTL_SECONDS viene rimosso.
class SubscribedVarCollection:
	var peer     : PacketPeerUDP       # peer UDP del subscriber
	var var_list : Dictionary = {}     # variabili sottoscritte
	var ttl      : float      = 0.0   # timer TTL (reset a ogni keep-alive)

	func init(p: PacketPeerUDP) -> void:
		peer = p

	## Incrementa il TTL. Restituisce true se il subscriber è scaduto.
	func process(delta: float) -> bool:
		ttl += delta
		return ttl > TTL_SECONDS

	## Reset del TTL — chiamato alla ricezione di un keep-alive.
	func keep_alive() -> void:
		ttl = 0.0

	func set_var_list(v: Dictionary) -> void:
		var_list = v

# ── Lifecycle ─────────────────────────────────────────────────────────────────

## Inizializzazione del server UDP e delle strutture dati.
## process_priority = -10 garantisce che DDS venga eseguito prima di tutti
## gli altri nodi, così i dati sono aggiornati prima che gli script li leggano.
func _ready() -> void:
	process_priority = -10   # esegui prima di tutti gli altri nodi
	udp_server = UDPServer.new()
	udp_server.listen(SERVER_PORT)
	subscribers  = {}
	variables    = {}
	_local_vars  = {}
	print("[DDS] Server UDP in ascolto su :", SERVER_PORT)

## Loop principale DDS — eseguito ogni frame.
## Gestisce: rimozione subscriber scaduti, nuove connessioni, ricezione pacchetti.
func _process(delta: float) -> void:
	# Rimuove i subscriber che non inviano keep-alive da TTL_SECONDS
	for k in subscribers.keys():
		if subscribers[k].process(delta):
			subscribers.erase(k)

	# Accetta nuove connessioni UDP
	udp_server.poll()
	if udp_server.is_connection_available():
		var peer = udp_server.take_connection()
		udp_peers.append(peer)

	# Svuota l'intera coda di pacchetti di ogni peer in un solo frame
	# per evitare accumulo di dati in caso di burst di pacchetti
	for peer in udp_peers:
		while peer.get_available_packet_count() > 0:
			var col: SubscribedVarCollection
			if not subscribers.has(peer):
				col = SubscribedVarCollection.new()
				col.init(peer)
				subscribers[peer] = col
			else:
				col = subscribers[peer]

			var pkt     = peer.get_packet()
			var command = pkt.decode_u8(0)
			match command:
				COMMAND_KEEP_ALIVE: col.keep_alive()
				COMMAND_SUBSCRIBE:  _subscribe_from_remote(peer, pkt)
				COMMAND_PUBLISH:    _publish_from_remote(pkt)

# ── Ricezione pacchetti ───────────────────────────────────────────────────────

## Decodifica un pacchetto PUBLISH dal bridge e aggiorna _local_vars.
## Formato pacchetto: [0x82][tipo][len_name][name...][valore]
##
## TickId è gestito separatamente: aggiorna i timestamp e non va in _local_vars.
## TickId < 0 indica che Alvik non sta inviando dati (stale).
func _publish_from_remote(pkt: PackedByteArray) -> void:
	var typ    = pkt.decode_u8(1)
	var nlen   = pkt.decode_u8(2)
	var name1  = pkt.slice(3, 3 + nlen).get_string_from_utf8()
	var offset = 3 + nlen
	var val

	match typ:
		DDS_TYPE_INT:   val = pkt.decode_s32(offset)
		DDS_TYPE_FLOAT: val = pkt.decode_float(offset)
		_: return

	# TickId — aggiorna i timestamp per il rilevamento della connessione
	# TickId è un metadato di protocollo, non un dato applicativo. 
	# Ha un ruolo speciale:
	# 
	# 1. Rileva la connessione — last_receive_msec viene aggiornato 
	#    solo quando arriva un TickId valido, permettendo a digital_twin_v1_7.gd 
	#    di sapere se Alvik è connesso (Time.get_ticks_msec() - DDS.last_receive_msec < 2000)
	#
	# 2. TickId < 0 = stale — quando il bridge non riceve dati da Alvik invia TickId = -1. 
	#    Se fosse in _local_vars, gli script potrebbero leggere -1 come valore numerico e 
	#    usarlo per calcoli errati
	#
	# 3. Sincronizzazione — current_tick_id e last_valid_tick sono usati da tick_age() 
	#    per misurare quanti tick sono passati senza dati, indipendentemente dalle variabili 
	#    applicative
	#
	# In pratica se TickId finisse in _local_vars, qualsiasi script potrebbe fare DDS.read("TickId") 
	# e ottenere valori come -1 o numeri molto grandi, senza capire che non è una misura fisica ma 
	# un contatore di protocollo.
	
	if name1 == "TickId":
		var incoming = int(val)
		if incoming < 0:
			push_warning("[DDS] Alvik stale")
		else:
			current_tick_id   = incoming
			last_valid_tick   = incoming
			last_receive_msec = Time.get_ticks_msec()
		return   # TickId non va in _local_vars

	_local_vars[name1] = val

## Processa una richiesta di sottoscrizione da un peer remoto.
## Registra le variabili richieste e associa il peer come subscriber.
func _subscribe_from_remote(peer: PacketPeerUDP, pkt: PackedByteArray) -> void:
	var n_vars = pkt.decode_u8(1)
	var idx    = 2
	var pv     = {}
	for i in range(n_vars):
		var nlen  = pkt.decode_u8(idx)
		var vname = pkt.slice(idx + 1, idx + 1 + nlen).get_string_from_utf8()
		if not variables.has(vname):
			var v = DDSVariable.new()
			v.init(vname)
			variables[vname] = v
		variables[vname].add_peer(peer)
		pv[vname] = variables[vname]
		idx += 1 + nlen
	subscribers[peer].set_var_list(pv)

# ── API pubblica ──────────────────────────────────────────────────────────────

## Registra interesse per una variabile DDS.
## Dopo la sottoscrizione il valore è leggibile con read().
## Inizializza il valore a null finché non arriva il primo dato.
func subscribe(var_name: String) -> void:
	_local_vars[var_name] = null
# subscribe() semplicemente aggiunge la chiave a _local_vars con valore null
# così quando arriva un pacchetto DDS con quel nome, _publish_from_remote() 
# trova la chiave e aggiorna il valore. Se una variabile non è stata sottoscritta,
# il pacchetto viene comunque ricevuto e scritto in _local_vars perché non c'è 
# nessun filtro — subscribe() serve solo a pre-inizializzare il valore a null 
# per evitare che DDS.read() restituisca un errore prima del primo pacchetto.
# La lista delle variabili di interesse è in digital_twin_v1_7.gd nel metodo _ready()

## Legge il valore corrente di una variabile DDS.
## Restituisce null se la variabile non è stata ricevuta.
## Esempio: var x = DDS.read("X")
## (Restituendo sempre e comunque un valore si evitano crash)
func read(var_name: String):
	return _local_vars.get(var_name)

## Pubblica una variabile DDS verso tutti i subscriber registrati.
## Usato internamente per inviare dati dal Digital Twin al bridge.
## Nel progetto viene usata una sola volta in digital_twin_v1_7.gd:
## gdscriptDDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)
func publish(var_name: String, type: int, value) -> void:
	if not variables.has(var_name):
		var v = DDSVariable.new()
		v.init(var_name)
		variables[var_name] = v
	variables[var_name].set_value(type, value)
	variables[var_name].publish()

## Restituisce l'età del tick corrente rispetto all'ultimo valido.
## Usato per rilevare perdita di dati (Alvik stale o disconnesso).
func tick_age() -> int:
	return current_tick_id - last_valid_tick
