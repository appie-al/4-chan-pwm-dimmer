//
// PWM Dimmer, max 4 channels
// Hardware ESP8266

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <EEPROM.h>

#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
const char* update_host = "esp8266-webupdate";

const char* Version = "1.7.8.4";
const char* compile_date = __DATE__ " " __TIME__;
#define nr_networks 2
#define preference_network 0  // this is suppposed to be the network for normal operation
#define nr_channels_max 4
#define uses_config_version 102
#define max_gpio 17  // GPIO16 is last one
#define GPIO_INPUT 1
#define GPIO_OUTPUT 2
#define network_timeout_time 10 // seconds
#define max_network_timeouts 720 // = 720 * 10 each = 2 hours
#define mqtt_timeout_time 2 // seconds
#define max_mqtt_timeouts 10
#define keep_alive_frequency 1800 // 30 minutes
#define mqtt_buffer_elements 20 // number of elemeents in the circular MQTT receive buffer
#define mqtt_max_receive_length 50 // maximum payload lengte van een ontvangen MQTT bericht
#define mqtt_max_queuename_length 15 // maximale lengte van de input mqtt-queue-names, include \0 terminator

void read_config_from_eeprom_2_ram(struct configuration *);
void handle_config(char *);
void smooth_dim_ticker_interrupt();
void my_sscanf (char *, byte, byte **);
void save_config2eeprom();
void switch_poll_ticker_interrupt();
void network_ticker_interrupt();
void keep_alive_ticker_interrupt();

struct switch_debounce
  { byte active;
    byte last;
    byte status;
    byte debounced_data;
  } sw[max_gpio]; // all possible GPIO's

byte gpio_pins[max_gpio];

struct rotary_switch_stuff
  { byte rotary_active;
    byte rotary_value;
    byte last_processed_rotary_value;  // laatste goed-verwerkte waarde. Als deze afwijkt van rotary_value dan iets doen
    byte pin_a;
    byte pin_b;
    byte rotary_value_change_per_pulse;
    int enc_position;
    signed int enc_value;
    int old_enc_value;
    byte switch_active;
    byte switch_pin;
    byte switch_type; // pulse(0 or 2) or on/off (1 or 3), 0,1 are connected to internal dimmer-value. 2,3 only send 'toggle' command over confirm channel
    byte last_processed_switch_value;  // laatste goed-verwerkte waarde.
  } rot_sw_inf[nr_channels_max];

struct wifi_nw
  {
    char ssid [15];
    char pwd [40];
    char ip_mqtt_server[16]; // nieuw
    byte valid;
  };

struct channel_values
  {
    byte output_pin;
    byte invert_value;
    byte output_enabled;
    byte rotary_1_pin;
    byte rotary_2_pin;
    byte rotary_value_change_per_pulse;
    byte rotary_enabled;
    byte on_off_switch_pin;
    byte on_off_switch_type; // pulse(0 or 2) or on/off (1 or 3), 0,1 are connected to internal dimmer-value. 2,3 only send 'toggle' command over confirm channel
    byte on_off_switch_enable;
    byte led_indicator_pin;
    byte led_indicator_enabled;
    byte min_value;
    byte max_value;
    byte low_dimmed_value;
  };

struct configuration
  { byte config_version;
    struct wifi_nw wifi_network[nr_networks]; // first is config network, next is working network
    struct channel_values channel[nr_channels_max];
    byte nr_active_channels;
    int dimspeed_command;
    int dimspeed_manual;
    int pwm_frequency;
    byte max_range;
    char my_id[10];
    byte flash_speed; // this is a ticker_milliseconds value
    byte flash_duration; // number of times to switch lamp on and off. Value of 20 means a duration of 20 * <flash_speed> (30) = 600 ms. Maximum value: 250
    byte flash_min_val; // low intensity setting
    byte flash_max_val; // high intensity setting

  } process_config, shadow_config;

// MQTT queues
char device_dim_1[nr_channels_max][mqtt_max_queuename_length];
char device_confirmation_topic[nr_channels_max][mqtt_max_queuename_length];
char device_status_topic [mqtt_max_queuename_length];
char device_config_topic [mqtt_max_queuename_length];

// MQTT circulal receiving buffer
char mqtt_circ_buffer_payload [mqtt_buffer_elements][mqtt_max_receive_length];
char mqtt_circ_buffer_channel [mqtt_buffer_elements][mqtt_max_queuename_length];
byte mqtt_mesg_status [mqtt_buffer_elements];
byte curr_read = 0;
byte curr_write = 0; // points to the next writable entry

const int mqttPort = 1883;
byte network_status = 0;

WiFiClient mijn_wifi_ding;
PubSubClient mqtt_client(mijn_wifi_ding);
Ticker smooth_dim_ticker;
Ticker switch_poll_ticker;
Ticker network_ticker;
Ticker keep_alive_ticker;

char local_ip[16];
char gateway_ip[16];
char device_hostname[10];

byte firmware_update_start = 0;
byte config_has_been_updated;

char mqtt_clientName[10];
int loopcount;
byte end_dim_value[nr_channels_max];
byte current_dim_value[nr_channels_max];
byte saved_dim_value[nr_channels_max];
byte ind_value_changed[nr_channels_max];
byte active_network = 0;
byte smooth_dim_ticker_status = 0; // 0:detached 1:attached
static byte smooth_dim_ticker_tick = 0;
byte switch_poll_ticker_status = 0; // 0:detached 1:attached
static byte switch_poll_ticker_tick = 0;
byte network_ticker_status = 0; // 0:detached 1:attached
static byte network_ticker_tick = 0;
static byte keep_alive_ticker_tick = 0;
int network_timeout_counter;
int mqtt_timeout_counter;

byte timer_request = 0;

byte flash_requested; // if active than this is <channel + 1>
byte flash_count; // duration_counter, 0xff is initial setting
byte flash_pin;

uint32_t dimspeed_override;

/////////////////////////////Rotary Encoder Vars//////////////////////////////////////
signed char enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

void shutdown()
{
  //Serial.println("Shutting down.");
  ESP.deepSleep(0);
  //Serial.println("Shutdown failed.");
}

void restart()
{ //Serial.println("Restarting..");
  ESP.restart();
  //Serial.println("Restart failed.");
}

void reset_board()
{
  //Serial.println("Reseting..");
  ESP.reset();
  //Serial.println("Reset failed.");
}

void start_update_server()
{
  firmware_update_start = 1;
}

void mqtt_callback(char* channel, byte* message_raw, unsigned int length)
{
byte i;
char message[100];
char outline[80];

for (i=0;i < length; i++) message[i] = (char)message_raw[i];
message[i] = 0;

//Serial.println(channel);
//Serial.println(message);

if (strlen(message) > mqtt_max_receive_length-1)
  { // fout, te lang bericht
    sprintf (outline, "Incoming message too big: %d, max %d possible", strlen(message), mqtt_max_receive_length-1);
    mqtt_client.publish(device_status_topic, outline);
    return;
  }

if (mqtt_mesg_status[curr_write])
  { // fout, geen ruimte !!
    mqtt_client.publish(device_status_topic, "No space to save incoming MQTT message");
  }
 else
  {
    strcpy (mqtt_circ_buffer_channel[curr_write], channel);
    strcpy (mqtt_circ_buffer_payload[curr_write], message);
    mqtt_mesg_status[curr_write] = 1; // mark it as 'not processed yet'
    if (++curr_write == mqtt_buffer_elements)  // reached end of buffer ?
      {
        curr_write = 0;
      }
  }
}

void setup()
{// byte i;

//Serial.begin(115200);
delay(2000);
//Serial.println();
//Serial.println("Starting");

// boot-time only

memset (mqtt_mesg_status, 0, sizeof (mqtt_mesg_status)); // nulstellen
keep_alive_ticker.attach(keep_alive_frequency, keep_alive_ticker_interrupt);

// reusable init code:

read_eeprom2ram();
read_config_from_eeprom_2_ram(&process_config);
read_config_from_eeprom_2_ram(&shadow_config);
init_gpio_input_output_table();
prepare_and_fill_structs();
set_hostname();
set_pwm_settings();
init_outputs_and_define_channel_queues();
}

void loop()
{
  do_network_mqtt_things();

  if (firmware_update_start)
    {
      do_firmware_upgrade();
    }
   else
    {
      if (flash_requested)
        {
          flash_a_light();
        }
       else
        {
          smooth_dim_the_lights();
        }
      manage_poll_switches();
      handle_rotary();
      act_on_rotary();
      handle_and_act_on_off_switches();
      do_process_mqtt_incoming();
      if (keep_alive_ticker_tick) send_keep_alive();
    }
}

void do_firmware_upgrade()
{
  switch (firmware_update_start)
    { case 1:  init_outputs_and_define_channel_queues();  // just switch off all lights
               //Serial.println("Starting update server..");
               MDNS.begin(update_host);
               httpUpdater.setup(&httpServer);
               httpServer.begin();
               MDNS.addService("http", "tcp", 80);
               firmware_update_start = 2;
               break;
      case 2:  httpServer.handleClient();
      default: break;
    }
}

void smooth_dim_the_lights()
{
char outline[80];
byte i;
byte value_was_changed;
byte val_end, val_curr;

for (i=0; i<process_config.nr_active_channels; i++) // check if we should start ticker..
  {
    if (current_dim_value[i] != end_dim_value[i])
      {
        if (!(smooth_dim_ticker_status))
          {
            smooth_dim_ticker_status = 1;
            smooth_dim_ticker.attach_ms(dimspeed_override ? dimspeed_override : process_config.dimspeed_command, smooth_dim_ticker_interrupt);
            dimspeed_override = 0;
            break;
          }
      }
  }

if (smooth_dim_ticker_tick)
  { value_was_changed = 0;
    smooth_dim_ticker_tick = 0;
    for (i=0; i<process_config.nr_active_channels; i++)
      { val_curr = current_dim_value[i];
        val_end = end_dim_value[i];
        if (val_end != val_curr)
          { if (val_curr < val_end)
              val_curr++;
             else
              val_curr--;

            current_dim_value[i] = val_curr;
            value_was_changed = 1;
            ind_value_changed[i] = 1;
            if (process_config.channel[i].output_enabled) analogWrite(process_config.channel[i].output_pin, process_config.channel[i].invert_value ? process_config.max_range - val_curr : val_curr);
          }
      }
    if (!(value_was_changed)) // this happens one time after all values are at the end-value
      { smooth_dim_ticker.detach();
        smooth_dim_ticker_status = 0;
        for (i=0; i<process_config.nr_active_channels; i++)
          { if (ind_value_changed[i])
              { ind_value_changed[i] = 0;
                sprintf (outline, "%d", current_dim_value[i]);
                mqtt_client.publish(device_confirmation_topic[i], outline);

                if (process_config.channel[i].led_indicator_enabled)
                  { if (current_dim_value[i])
                      digitalWrite(process_config.channel[i].led_indicator_pin,LOW);
                     else
                      digitalWrite(process_config.channel[i].led_indicator_pin,HIGH);
                  }
              }
          }
      }
  }
}

void read_eeprom2ram()
{
  EEPROM.begin(512);  // copy eeprom to volatile memory
}

void read_config_from_eeprom_2_ram(struct configuration *dest)
{
  int size = sizeof (struct configuration);
  char *p = (char *)dest;
  int i;

  memset (dest, 0, size); // clear memory
  if (uses_config_version == EEPROM.read(0)) //get config version number from EEPROM, is first byte
      for (i=0; i<size; i++) *p++ = EEPROM.read(i);
   else
      update_or_init_config(EEPROM.read(0), uses_config_version, dest);
}

void config_copy_shadow_2_process()
{
  int size = sizeof (struct configuration);
  char *p = (char *)&process_config;
  char *s = (char *)&shadow_config;
  int i;

  for (i=0; i<size; i++) *p++ = *s++;
}

void init_default_network(struct configuration *dest)
{
  strcpy (dest->wifi_network[0].ssid, "SSID1");
  strcpy (dest->wifi_network[0].pwd, "PWD1");
  strcpy (dest->wifi_network[0].ip_mqtt_server, "10.0.0.123");
  dest->wifi_network[0].valid = 1;

  strcpy (dest->wifi_network[1].ssid, "SSID2");
  strcpy (dest->wifi_network[1].pwd, "PWD2");
  strcpy (dest->wifi_network[1].ip_mqtt_server, "10.0.0.123");
  dest->wifi_network[1].valid = 1;
}

void init_default_config(struct configuration *dest)
{
  memset (dest, 0, sizeof (struct configuration)); // clear memory

  dest->config_version = uses_config_version;
  strcpy (dest->my_id, "dum/0");

  init_default_network(dest);

  dest->nr_active_channels = 0;
  dest->dimspeed_command = 10;
  dest->dimspeed_manual = 10;
  dest->pwm_frequency = 100;
  dest->max_range = 100;
  dest->flash_speed = 30;
  dest->flash_duration = 20;
  dest->flash_min_val = 20;
  dest->flash_max_val = 80;

  strcpy (mqtt_clientName, dest->my_id);  // werkt dat met '/' in de naam??
}

void handle_config(char *mesg)
{
char outline[80];
byte i;
byte *t[10];

//Serial.print("it's a config: ");
//Serial.println(mesg);

if (mesg[0] == '?')  display_all_config();

if (!(strncmp(mesg, "sub-status", 10))) display_status_subdevice(); // display status of all active sub devices

if (!(strncmp(mesg, "pwmfreq=", 8))) // set pwm frequency
  { shadow_config.pwm_frequency = atoi (&mesg[8]);
    analogWriteFreq(shadow_config.pwm_frequency);
    sprintf (outline, "PWM frequency changed to: %d", shadow_config.pwm_frequency);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "maxrange=", 9))) // set maximal dimming value (= next to 'full switched on')
  { shadow_config.max_range = atoi (&mesg[9]);
    analogWriteRange(shadow_config.max_range);
    sprintf (outline, "PWM max_range changed to: %d", shadow_config.max_range);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "flash_speed=", 12))) // set flash speed (on and off) in milliseconds
  { shadow_config.flash_speed = atoi (&mesg[12]);
    sprintf (outline, "Flash speed changed to: %d", shadow_config.flash_speed);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "flash_duration=", 15))) // set flash duration
  { shadow_config.flash_duration = atoi (&mesg[15]);
    sprintf (outline, "Flash duration changed to: %d", shadow_config.flash_duration);
    mqtt_client.publish(device_status_topic, outline);
  }
  if (!(strncmp(mesg, "flash_min_val=", 14))) // set flash duration
    { shadow_config.flash_min_val = atoi (&mesg[14]);
      sprintf (outline, "Flash low intensity changed to: %d", shadow_config.flash_min_val);
      mqtt_client.publish(device_status_topic, outline);
    }
if (!(strncmp(mesg, "flash_max_val=", 14))) // set flash duration
    { shadow_config.flash_max_val = atoi (&mesg[14]);
      sprintf (outline, "Flash high intensity changed to: %d", shadow_config.flash_max_val);
      mqtt_client.publish(device_status_topic, outline);
    }
if (!(strncmp(mesg, "dimspeedcommand=", 16))) // set dimming speed for external commands
  { shadow_config.dimspeed_command = atoi (&mesg[16]);
    sprintf (outline, "Dimming speed for commands changed to: %d", shadow_config.dimspeed_command);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "dimspeed_override=", 18))) // set dimming speed for external commands
  { dimspeed_override = atoi (&mesg[18]);
    sprintf (outline, "Dimming speed override: %d", dimspeed_override);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "id=", 3))) // set id of device, will be active after reboot
  { strcpy (shadow_config.my_id, &mesg[3]);
    sprintf (outline, "Device_id: %s", shadow_config.my_id);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "nr_channels=", 12))) // set number of active channels
  { shadow_config.nr_active_channels = atoi (&mesg[12]);
    sprintf (outline, "Active channels: %d", shadow_config.nr_active_channels);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "ssid0=", 6))) // set first wifi network ssid
  { strcpy (shadow_config.wifi_network[0].ssid, &mesg[6]);
    sprintf (outline, "First WiFi ssid: %s", shadow_config.wifi_network[0].ssid);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "network_pwd0=", 13))) // set first wifi network password. Zorg ervoor enkele quotes in de commandline van mosquitto_pub te gebruiken: mosquitto_pub -q 2 -t dim/2/cf -m 'network_pwd=blabli!ablablabla12334'
  { strcpy (shadow_config.wifi_network[0].pwd, &mesg[13]);
    sprintf (outline, "First WiFi password: %s", shadow_config.wifi_network[0].pwd);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "mqtt_ip0=", 9))) // set firstwifi network mqtt broker ip
  { strcpy (shadow_config.wifi_network[0].ip_mqtt_server, &mesg[9]);
    sprintf (outline, "First MQTT server: %s", shadow_config.wifi_network[0].ip_mqtt_server);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "ssid1=", 6))) // set second wifi network ssid
  { strcpy (shadow_config.wifi_network[1].ssid, &mesg[6]);
    sprintf (outline, "Second WiFi ssid: %s", shadow_config.wifi_network[1].ssid);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "network_pwd1=", 13))) // set second wifi network password. Zorg ervoor enkele quotes in de commandline van mosquitto_pub te gebruiken: mosquitto_pub -q 2 -t dim/2/cf -m 'network_pwd=blabli!ablablabla12334'
  { strcpy (shadow_config.wifi_network[1].pwd, &mesg[13]);
    sprintf (outline, "Second WiFi password: %s", shadow_config.wifi_network[1].pwd);
    mqtt_client.publish(device_status_topic, outline);
  }
if (!(strncmp(mesg, "mqtt_ip1=", 9))) // set second wifi network mqtt broker ip
  { strcpy (shadow_config.wifi_network[1].ip_mqtt_server, &mesg[9]);
    sprintf (outline, "Second MQTT server: %s", shadow_config.wifi_network[1].ip_mqtt_server);
    mqtt_client.publish(device_status_topic, outline);
  }

if (!(strncmp(mesg, "channel_set_output=", 19))) // set channel-specific things
  { t[0] = &i;
    my_sscanf (&mesg[19], 1, t);
    if (i < nr_channels_max)  // Don't want to write outside our table...
     {
      t[0] = &i;
      t[1] = &shadow_config.channel[i].output_enabled;
      t[2] = &shadow_config.channel[i].output_pin;
      t[3] = &shadow_config.channel[i].invert_value;
      t[4] = &shadow_config.channel[i].min_value;
      t[5] = &shadow_config.channel[i].max_value;
      t[6] = &shadow_config.channel[i].low_dimmed_value;

      my_sscanf (&mesg[19], 7, t);

      cfg_print_single_node_pwm_output(i);
     }
  }
if (!(strncmp(mesg, "channel_set_rotary=", 19))) // set channel-specific things
  { t[0] = &i;
    my_sscanf (&mesg[19], 1, t);
    if (i < nr_channels_max)  // Don't want to write outside our table...
     {
      t[0] = &i;
      t[1] = &shadow_config.channel[i].rotary_enabled;
      t[2] = &shadow_config.channel[i].rotary_1_pin;
      t[3] = &shadow_config.channel[i].rotary_2_pin;
      t[4] = &shadow_config.channel[i].rotary_value_change_per_pulse;

      my_sscanf (&mesg[19], 5, t);

      cfg_print_single_node_rotary(i);
     }
  }
if (!(strncmp(mesg, "channel_set_switch=", 19))) // set channel-specific things
  { t[0] = &i;
    my_sscanf (&mesg[19], 1, t);
    if (i < nr_channels_max)  // Don't want to write outside our table...
     {
      t[0] = &i;
      t[1] = &shadow_config.channel[i].on_off_switch_enable;
      t[2] = &shadow_config.channel[i].on_off_switch_pin;
      t[3] = &shadow_config.channel[i].on_off_switch_type;

      my_sscanf (&mesg[19], 4, t);

      cfg_print_single_node_switch(i);
     }
  }
if (!(strncmp(mesg, "channel_set_ledind=", 19))) // set channel-specific things
  { t[0] = &i;
    my_sscanf (&mesg[19], 1, t);
    if (i < nr_channels_max)  // Don't want to write outside our table...
     {
      t[0] = &i;
      t[1] = &shadow_config.channel[i].led_indicator_enabled;
      t[2] = &shadow_config.channel[i].led_indicator_pin;

      my_sscanf (&mesg[19], 3, t);

      cfg_print_single_node_indication_led(i);
     }
  }
if (!(strncmp(mesg, "save", 4))) // save to eeprom and reread eprom to config struct
  { save_config2eeprom();
  }

if (!(strncmp(mesg, "init_network", 12))) // only init network settings. Needed when a new image with new network settings is implemented
  { init_default_network(&shadow_config);
    mqtt_client.publish(device_status_topic, "Init network done, needs save and reset to get everything active");
  }

if (!(strncmp(mesg, "init_all", 8))) // get an empty config, except for network
  { init_default_config(&shadow_config);
    mqtt_client.publish(device_status_topic, "Init done, needs save and reset to get everything active");
  }

if (!(strncmp(mesg, "restart", 7))) // restarting device, doesn't always work well
  { mqtt_client.publish(device_status_topic, "Restarting now");
    // alle uitgangen uitzetten?
    restart();
  }
if (!(strncmp(mesg, "reset", 5))) // reset device
  { mqtt_client.publish(device_status_topic, "Reseting now");
    // alle uitgangen uitzetten?
    reset_board();
  }
if (!(strncmp(mesg, "update_firmware", 15))) // start the update web server
  { mqtt_client.publish(device_status_topic, "Starting webserver");
    start_update_server();
  }
if (!(strncmp(mesg, "activate_shadow_config", 22))) // activate the shadow config
  { mqtt_client.publish(device_status_topic, "Activate shadow config");
    activate_shadow_config();
  }
if (!(strncmp(mesg, "refresh_shadow_config", 21))) // refresh the shadow config from eeprom, very usefull when wanting to start over again
  { mqtt_client.publish(device_status_topic, "Refresh shadow config");
    read_config_from_eeprom_2_ram(&shadow_config);
    mqtt_client.publish(device_status_topic, "Refresh shadow config done");
  }
}

void save_config2eeprom()
{
char outline[80];
int size = sizeof (struct configuration);
char *p = (char *)&process_config;
int i;

if (size < 512) // max memsize
  {
    sprintf (outline, "Saving to EEPROM, %d bytes", size);
    mqtt_client.publish(device_status_topic, outline);
    for (i=0; i<size; i++) EEPROM.write(i, *p++);
    sprintf (outline, "Committing to EEPROM");
    mqtt_client.publish(device_status_topic, outline);
    EEPROM.commit(); // use commit to NOT free the ram copy of the eeprom content
    sprintf (outline, "Config saved to EEPROM, now reading in again and restoring to config");
    mqtt_client.publish(device_status_topic, outline);
    read_config_from_eeprom_2_ram(&process_config);
    read_config_from_eeprom_2_ram(&shadow_config);
    sprintf (outline, "EEPROM read from config done");
  }
 else
  {
    sprintf (outline, "Config is bigger than EEPROM size, not saving");
  }
mqtt_client.publish(device_status_topic, outline);
}

void my_sscanf (char *in, byte num, byte **var)
{
char own_buffer[100];  // should have our own copy because we're modifying it...
char *start = own_buffer;
char *eind = own_buffer;
byte i;

strcpy (own_buffer, in);

for (i=0; i<num; i++)
  {
    while (1)
      {
        if (*eind == ',' || *eind == 0) break;
        eind++;
      }

    *eind = 0; //terminate string
    *var[i] = atoi (start);
    start = eind+1;
    eind = start;
  }
}

void display_all_config()
{
  cfg_print_network();
  cfg_print_misc();
  cfg_print_nodes();
}

void display_status_subdevice()
{
  byte i;
  char outline[8];

  for (i=0; i<process_config.nr_active_channels; i++)
    {
      sprintf (outline, "%d", current_dim_value[i]);
      if (network_status == 100) mqtt_client.publish(device_confirmation_topic[i], outline);
    }
}

void cfg_print_misc()
{
  char outline[100];

    sprintf (outline, "Device %s is alive at %s, version %s, gateway is %s", shadow_config.my_id, local_ip, Version, gateway_ip);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Compiled: %s", compile_date);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "ID: %s", shadow_config.my_id);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Config version: %d", shadow_config.config_version);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Config size: %d", sizeof (struct configuration));
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Active channels: %d", shadow_config.nr_active_channels);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Dimspeed external command: %d", shadow_config.dimspeed_command);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Dimspeed manual: %d", shadow_config.dimspeed_manual);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "PWM frequency: %d", shadow_config.pwm_frequency);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "DIM max range: %d", shadow_config.max_range);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Flash speed: %d", shadow_config.flash_speed);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Flash duration: %d", shadow_config.flash_duration);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Flash low intensity: %d", shadow_config.flash_min_val);
    mqtt_client.publish(device_status_topic, outline);
    sprintf (outline, "Flash high intensity: %d", shadow_config.flash_max_val);
    mqtt_client.publish(device_status_topic, outline);
}

void cfg_print_network()
{
  char outline[100];
  byte i;
  for (i=0; i<nr_networks; i++)
    { sprintf (outline, "Network %d: SSID:%s, PWD:%s, MQTT-server: %s, valid:%d", i, shadow_config.wifi_network[i].ssid, shadow_config.wifi_network[i].pwd, shadow_config.wifi_network[i].ip_mqtt_server, shadow_config.wifi_network[i].valid);
      mqtt_client.publish(device_status_topic, outline);
    }
}

void cfg_print_nodes()
{
  byte i;

  for (i=0; i<nr_channels_max; i++) cfg_print_single_node(i);
}

void cfg_print_single_node(byte channel_nr)
{
  cfg_print_single_node_pwm_output(channel_nr);
  cfg_print_single_node_rotary(channel_nr);
  cfg_print_single_node_switch(channel_nr);
  cfg_print_single_node_indication_led(channel_nr);
}

void cfg_print_single_node_pwm_output(byte channel_nr)
{
  char outline[100];

  sprintf (outline, "channel output (%d): enabled:%d, pin:%d, invert:%d, min:%d, max:%d, low-dim:%d" \
                        , channel_nr \
                        , shadow_config.channel[channel_nr].output_enabled \
                        , shadow_config.channel[channel_nr].output_pin \
                        , shadow_config.channel[channel_nr].invert_value \
                        , shadow_config.channel[channel_nr].min_value \
                        , shadow_config.channel[channel_nr].max_value \
                        , shadow_config.channel[channel_nr].low_dimmed_value \
             );
      mqtt_client.publish(device_status_topic, outline);
}

void cfg_print_single_node_rotary(byte channel_nr)
{
  char outline[100];

  sprintf (outline, "channel rotary (%d): enabled:%d, pin1:%d, pin2:%d, step_per_pulse:%d" \
                  , channel_nr \
                  , shadow_config.channel[channel_nr].rotary_enabled \
                  , shadow_config.channel[channel_nr].rotary_1_pin \
                  , shadow_config.channel[channel_nr].rotary_2_pin \
                  , shadow_config.channel[channel_nr].rotary_value_change_per_pulse \
             );
  mqtt_client.publish(device_status_topic, outline);
}

void cfg_print_single_node_switch(byte channel_nr)
{
  char outline[100];

  sprintf (outline, "channel switch (%d): enabled:%d, pin:%d, type:%d" \
                , channel_nr \
                , shadow_config.channel[channel_nr].on_off_switch_enable \
                , shadow_config.channel[channel_nr].on_off_switch_pin \
                , shadow_config.channel[channel_nr].on_off_switch_type \
             );
  mqtt_client.publish(device_status_topic, outline);
}

void cfg_print_single_node_indication_led(byte channel_nr)
{
  char outline[100];

  sprintf (outline, "channel indled (%d): enabled:%d, pin:%d" \
                , channel_nr \
                , shadow_config.channel[channel_nr].led_indicator_enabled \
                , shadow_config.channel[channel_nr].led_indicator_pin \
             );

   mqtt_client.publish(device_status_topic, outline);
}

void prepare_and_fill_structs()
{
  byte i;
  struct rotary_switch_stuff *one_switch;

  memset (&sw, 0, sizeof (sw)); // clear memory
  memset (&rot_sw_inf, 0, sizeof (rot_sw_inf)); // clear memory

  for (i=0,one_switch=&rot_sw_inf[0]; i<process_config.nr_active_channels; i++,one_switch++)
    { if (process_config.channel[i].on_off_switch_enable)
        {
          one_switch->switch_active = 1;
          one_switch->switch_pin = process_config.channel[i].on_off_switch_pin;
          one_switch->switch_type = process_config.channel[i].on_off_switch_type;
          set_input_pullup(one_switch->switch_pin);
          one_switch->last_processed_switch_value = sw[one_switch->switch_pin].debounced_data;
        }
      if (process_config.channel[i].rotary_enabled)
        {
          one_switch->rotary_active = 1;
          one_switch->pin_a = process_config.channel[i].rotary_1_pin;
          one_switch->pin_b = process_config.channel[i].rotary_2_pin;
          one_switch->rotary_value_change_per_pulse = process_config.channel[i].rotary_value_change_per_pulse;
          set_input_pullup(one_switch->pin_a);
          set_input_pullup(one_switch->pin_b);
        }
    }
}

void set_input_pullup(byte pin)
{
  pinMode(pin, INPUT_PULLUP);
  delay (5);
  sw[pin].last = sw[pin].debounced_data = digitalRead(pin);
  sw[pin].active = 1;
  gpio_pins[pin] = GPIO_INPUT;

  //Serial.print("Inputpin defined: ");
  //Serial.println(pin);
}

void set_output(byte pin)
{
  pinMode(pin, OUTPUT);
  gpio_pins[pin] = GPIO_OUTPUT;
}

void poll_switches()
{
  byte i;
  byte current;

  timer_request = 0;

  for (i=0; i<max_gpio; i++)
    {
      if (sw[i].active)
        {
          current = digitalRead(i);
          switch (sw[i].status)
            { case 0  : if (current != sw[i].last) { sw[i].status = 1;
                                                     sw[i].last = current;
                                                     timer_request = 1;
                                                   }
                        break;
              case 1  : if (current == sw[i].last) { sw[i].status = 0;
                                                     sw[i].debounced_data = current;
//                                                     sw[i].changed = 1;
                                                   }
                                                  else
                                                   {
                                                     sw[i].status = 2;
                                                     timer_request = 1;
                                                   }
                        break;
              case 2  : sw[i].status = 0;
                        sw[i].debounced_data = current;
                        sw[i].last = current;
//                        sw[i].changed = 1;
                        break;
              default : break;
            }
        }
    }

 if (timer_request)
   { if (!switch_poll_ticker_status)
       { switch_poll_ticker_status = 1;
         switch_poll_ticker.attach_ms(3, switch_poll_ticker_interrupt); // 8 milliseconds
//         //Serial.println("+");
       }
   }
  else
   {
     if (switch_poll_ticker_status)
       { switch_poll_ticker.detach();
         switch_poll_ticker_status = 0;
//         //Serial.println("-");
       }
   }
}

void switch_poll_ticker_interrupt()
{
  switch_poll_ticker_tick = 1;
}

void network_ticker_interrupt()
{
  network_ticker_tick = 1;
}

void keep_alive_ticker_interrupt()
{
  keep_alive_ticker_tick = 1;
}

void smooth_dim_ticker_interrupt()
{
  smooth_dim_ticker_tick = 1;
}

void manage_poll_switches()
{
 if (!switch_poll_ticker_status)
   { poll_switches();  // just poll un-timed until one of the switches changed.
   }
  else // ticker is running in order to wait some ms to deny bouncing of contacts
   { if (switch_poll_ticker_tick)
       { poll_switches();
         switch_poll_ticker_tick=0;
       }
   }
}

void act_on_rotary()
{
 struct rotary_switch_stuff *one_switch;
 byte i;

 for (i=0,one_switch=&rot_sw_inf[0]; i<process_config.nr_active_channels; i++,one_switch++)
  {
     if (one_switch->rotary_active)
       {
         if (one_switch->last_processed_rotary_value != one_switch->rotary_value)
           {
             //Serial.print("Rotary(");
             //Serial.print(i);
             //Serial.print(") changed: ");
             //Serial.println(one_switch->rotary_value,DEC);
             one_switch->last_processed_rotary_value = one_switch->rotary_value;
             end_dim_value[i] = one_switch->rotary_value;
           }
       }
  }
}

void update_rotary (byte channel)
{
  if (process_config.channel[channel].rotary_enabled)
    { struct rotary_switch_stuff *one_switch = &rot_sw_inf[channel];
      one_switch->last_processed_rotary_value = one_switch->rotary_value = end_dim_value[channel];
      if (end_dim_value[channel])
        {
          one_switch->old_enc_value = one_switch->enc_value = max((one_switch->rotary_value / one_switch->rotary_value_change_per_pulse),1) * 4;          
        }
       else
        {
          one_switch->old_enc_value = one_switch->enc_value = 0;
        }
    }
}

void handle_and_act_on_off_switches()
{
 struct rotary_switch_stuff *one_switch;
 byte i;

 for (i=0,one_switch=&rot_sw_inf[0]; i<process_config.nr_active_channels; i++,one_switch++)
  {
    if (one_switch->switch_active)
      {
        if (one_switch->last_processed_switch_value != sw[one_switch->switch_pin].debounced_data) // switch changed?
        {
          one_switch->last_processed_switch_value = sw[one_switch->switch_pin].debounced_data;
          if ((one_switch->switch_type  & 0x01) == 0) // pulse
            {
              if (!one_switch->last_processed_switch_value) //pushed, logic low !!!
                {
                  set_channel_output_toggle (i, (one_switch->switch_type & 0x02));
                }
            }
           else // switch type is on/off
            {
              set_channel_output_toggle (i, (one_switch->switch_type & 0x02));
            }
        }
      }
  }
}

void set_channel_output_toggle (byte channel, byte send_msg_only)
{
  if (send_msg_only)
    {
       mqtt_client.publish(device_confirmation_topic[channel], "toggle");
    }
   else
    {
      if (end_dim_value[channel])
        {
          set_channel_output_off(channel);
        }
       else
        {
          set_channel_output_on(channel);
        }
    }
}

void set_channel_output_on (byte channel)
{
  if (!end_dim_value[channel]) // only if status now == off
    {
      end_dim_value[channel] = saved_dim_value[channel];
    }
}

void set_channel_output_off (byte channel)
{
  if (end_dim_value[channel]) // only if status now == on
    {
      saved_dim_value[channel] = end_dim_value[channel];
      end_dim_value[channel] = 0;
    }
}

void handle_rotary()
{
 struct rotary_switch_stuff *one_switch;
 byte input;
 byte i;
 byte temp_value;

 for (i=0,one_switch=&rot_sw_inf[0]; i<process_config.nr_active_channels; i++,one_switch++)
  {
     if (one_switch->rotary_active)
       {
         input = 0;
         if (sw[one_switch->pin_a].debounced_data) input |= 0x01;
         if (sw[one_switch->pin_b].debounced_data) input |= 0x02;

         one_switch->enc_position <<= 2;                       //remember previous state by shifting the lower bits up 2
         one_switch->enc_position |= ( input );
         one_switch->enc_value += enc_states[( one_switch->enc_position & 0x0f )];     // the lower 4 bits of old_AB & 16 are then
                                                        // the index for enc_states

         if (one_switch->enc_value == one_switch->old_enc_value)
           {
             continue;
           }

         if (one_switch->enc_value <= 0 )
           {
             one_switch->enc_value = 0;
           }

         if( one_switch->enc_value >= (400 / one_switch->rotary_value_change_per_pulse))
           {               // Arbitrary max value for testing purposes
             one_switch->enc_value = (400 / one_switch->rotary_value_change_per_pulse);
           }
         one_switch->old_enc_value = one_switch->enc_value;
         temp_value = (one_switch->enc_value/4) * one_switch->rotary_value_change_per_pulse;
         if (temp_value > 100) temp_value = 100;
         one_switch->rotary_value = temp_value;
       }
  }
}

void init_outputs_and_define_channel_queues()
{
byte i;

for (i=0; i<process_config.nr_active_channels; i++)
  { end_dim_value[i] = 0;
    current_dim_value[i] = 0;
    saved_dim_value[i] = process_config.channel[i].low_dimmed_value;

    if (process_config.channel[i].output_enabled)
      { //Serial.print("init pwm-output pin: ");
        //Serial.println(process_config.channel[i].output_pin);
        set_output(process_config.channel[i].output_pin);
        analogWrite(process_config.channel[i].output_pin, process_config.channel[i].invert_value ? process_config.max_range : 0);
      }

 if (process_config.channel[i].led_indicator_enabled)
      {
        set_output(process_config.channel[i].led_indicator_pin);
        digitalWrite(process_config.channel[i].led_indicator_pin,HIGH);
      }

// define incoming queue names
    sprintf (device_dim_1[i], "%s/%d/dim", process_config.my_id, i);
    //Serial.print("input queue: ");
    //Serial.println(device_dim_1[i]);

// define confirmation queue names
    sprintf (device_confirmation_topic[i], "%s/%d/cfr", process_config.my_id, i);
    //Serial.print("confirmation queue: ");
    //Serial.println(device_confirmation_topic[i]);
  }

// define outgoing status and incoming config channel names
sprintf (device_status_topic, "%s/st", process_config.my_id);
sprintf (device_config_topic, "%s/cf", process_config.my_id);
}

void set_pwm_settings()
{
analogWriteRange(process_config.max_range);
analogWriteFreq(process_config.pwm_frequency);
}

void do_network_mqtt_things()
{
  int ret;

  switch (network_status) // remember to put the mostly used status as first case !!
    {
      case 100: // Final state, everything is ok
               if (!mqtt_client.connected())
                 { network_status = 12; // just stay connected to the same network and connect again to MQTT
                 }
               if ((WiFi.status()!= WL_CONNECTED))
                 { network_status = 0; // just connect to the same network again and to the hole stuff over again
                   break;
                 }
               mqtt_client.loop();
               break;

      case  0: // just restarted
               WiFi.mode(WIFI_STA);
               WiFi.hostname(device_hostname); // DHCP Hostname (useful for finding device for static lease)
               //Serial.println("WiFi constants set");
               network_timeout_counter = 0;
               network_status = 1;
               break;

      case  1: // init connection
               ret = WiFi.disconnect();
               //Serial.print("return status of WiFi.disconnect: ");
               //Serial.println(ret, DEC);
               //Serial.print("Preparing SSID: ");
               //Serial.println(process_config.wifi_network[active_network].ssid);
               ret = WiFi.begin(process_config.wifi_network[active_network].ssid, process_config.wifi_network[active_network].pwd);
               //Serial.print("return status of WiFi.begin: ");
               //Serial.println(ret, DEC);
               network_ticker_status = 1;
               network_ticker_tick = 0;
               network_ticker.attach(network_timeout_time, network_ticker_interrupt);
               network_status = 3;
               break;

      case  3: // waiting for connection
               if (WiFi.status() == WL_CONNECTED)
                 {
                  network_ticker_status = 0;
                  network_ticker_tick = 0;
                  network_ticker.detach();
                  network_status = 10;
                  //Serial.println ("case 3: Connected!");
                  break;
                 }
                else
                 {
                   if (network_ticker_tick) // timeout raised
                     {
                       if (++network_timeout_counter == max_network_timeouts)
                         { network_status = 99;
                         }
                        else
                         { network_status = 5;
                           network_ticker_status = 0;
                           network_ticker.detach();
                         }
                       //Serial.print("Network timeout received, count: ");
                       //Serial.println(network_timeout_counter);
                     }
                    // no timeout yet, just keep waiting
                 }
               break;

      case  5: // prepare for next network
               //Serial.println("Trying next network");
               if (++active_network == nr_networks) active_network = 0;
               network_status = 1;
               break;

      case 10: // Connected to network, now prepare MQTT connection
               //Serial.print("Connected to SSID: ");
               //Serial.println(process_config.wifi_network[active_network].ssid);
               sprintf(local_ip, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
               sprintf(gateway_ip, "%d.%d.%d.%d", WiFi.gatewayIP()[0], WiFi.gatewayIP()[1], WiFi.gatewayIP()[2], WiFi.gatewayIP()[3] );
               //Serial.println("Prepare MQTT");
//               mqtt_client.setServer(active_network ? process_config.wifi_network[active_network].ip_mqtt_server : gateway_ip, mqttPort); // if setup-network (active_network==0) then use the accesspoints ip, else take it from the config
               mqtt_client.setServer(process_config.wifi_network[active_network].ip_mqtt_server, mqttPort);
               mqtt_client.setCallback(mqtt_callback);
               network_status = 12;
               break;

      case 12: // make MQTT connection and set timer
               //Serial.print("Connecting to MQTT: ");
               mqtt_timeout_counter = 0;
               network_ticker_status = 1;
               network_ticker_tick = 0;
               network_ticker.attach(network_timeout_time, network_ticker_interrupt);
               ret = mqtt_client.connect(mqtt_clientName);
               //Serial.print("return status of mqtt_client.connect: ");
               //Serial.println(ret, DEC);
               network_status = 14;
               break;

      case 14: // waiting for MQTT connection
               if (mqtt_client.connected())
                 {
                  network_ticker_status = 0;
                  network_ticker_tick = 0;
                  network_ticker.detach();
                  network_status = 50;
                  //Serial.println ("case 14: MQTT Connected!");
                  break;
                 }
                else
                 {
                   if (network_ticker_tick) // timeout raised
                     {
                       if (++mqtt_timeout_counter == max_mqtt_timeouts)
                         { network_status = 5; // maybe another network will have a broker running?
                           //Serial.println("Too many timeouts on MQTT connection attempts, trying other network");
                         }
                        else
                         { // retry connect
                           ret = mqtt_client.connect(mqtt_clientName);
                           //Serial.print("return status of mqtt_client.connect: ");
                           //Serial.println(ret, DEC);
                         }
                     }
                    // no timeout yet, just keep waiting
                 }
               break;

      case 50: // All connected, report myself to the broker and subscribe to queues
               { byte i;
                 char outline[100];

                 //Serial.println("MQTT connected");
                 sprintf (outline, "Device %s is alive at %s, version %s, gateway is %s", process_config.my_id, local_ip, Version, gateway_ip);
                 mqtt_client.publish(device_status_topic, outline);
                 sprintf (outline, "Compiled on %s", compile_date);
                 mqtt_client.publish(device_status_topic, outline);
                 mqtt_client.subscribe(device_config_topic);

                 for (i=0; i<process_config.nr_active_channels; i++)
                   { mqtt_client.subscribe(device_dim_1[i]);
                     //Serial.print("Subscribe to: ");
                     //Serial.println(device_dim_1[i]);
                   }

                 if (config_has_been_updated)
                   {
                     sprintf (outline, "Config has been %s to version %d", config_has_been_updated == 1 ? "upgraded" : "freshly initialized", uses_config_version);
                     mqtt_client.publish(device_status_topic, outline);
                     config_has_been_updated = 0;
                   }

                 network_status = 100; // final state
               }
               break;

      case 99: // many, many timeouts where raised
               //Serial.println("Sorry, had to reset for too much network timeouts");
               reset_board();  // maybe this helps
               break;

      default: break;
    }
}

void send_keep_alive()
{
  keep_alive_ticker_tick = 0;
  mqtt_client.publish(device_status_topic, "Alive");
  // test if still connected to preference-network. If not then try to connect to it..
  if (active_network != preference_network)
    {
      mqtt_client.publish(device_status_topic, "Connecting to prefered network again"); // maybe this will not be sent anymore because of netwerk-switching
      active_network = 0;
      network_status = 1;
    }
}

void reset_all_pins_to_default_inputs()
{
  byte i;

  for (i=0; i<max_gpio; i++)
    switch (gpio_pins[i])
      { case 0           : break;
        case GPIO_INPUT  : pinMode(i, INPUT);
                           break;
        case GPIO_OUTPUT : analogWrite(i,0); // make sure no pwm connected
                           pinMode(i, INPUT);
                           break;
        default          : break;
      }
}

void activate_shadow_config()
{
  reset_all_pins_to_default_inputs();
  init_gpio_input_output_table();
  config_copy_shadow_2_process();
  prepare_and_fill_structs();
  set_hostname();
  init_outputs_and_define_channel_queues();
  set_pwm_settings();
  network_status = 0;
  active_network = 0;
}

byte read_mqtt_msg_from_buffer (char *channel, char *out)
{
  if (mqtt_mesg_status[curr_read]) // if data available
    {
      strcpy (channel, mqtt_circ_buffer_channel[curr_read]);
      strcpy (out, mqtt_circ_buffer_payload[curr_read]);
      mqtt_mesg_status[curr_read] = 0;
      if (++curr_read == mqtt_buffer_elements)  // reached end of buffer
        {
          curr_read = 0;
        }
      return 1;
    }
   else
    { return 0;  // no data available
    }
}

void do_process_mqtt_incoming()
{
  byte i;
  char channel[mqtt_max_queuename_length]; // deze misschien globaal of static maken omdat snellere definitie (static vs stack)
  char mesg[mqtt_max_receive_length]; // deze misschien globaal  of static maken omdat snellere definitie (static vs stack)

  if (read_mqtt_msg_from_buffer (channel, mesg))
    { // test if it's a dim command
      for (i=0; i<process_config.nr_active_channels; i++)
        { if (!(strcmp(channel, device_dim_1[i])))
            { if (!strcmp(mesg, "on"))
                {
                  set_channel_output_on(i);
                  update_rotary (i);
                }
               else
                { if (!strcmp(mesg, "off"))
                    {
                      set_channel_output_off(i);
                      update_rotary (i);
                    }
                   else
                    { if (!strcmp(mesg, "toggle"))
                        {
                          set_channel_output_toggle(i,0);
                          update_rotary (i);
                        }
                       else
                        { if (!strcmp(mesg, "flash"))
                            {
                              if (process_config.channel[i].output_enabled)
                                {
                                  flash_requested = i+1;
                                  flash_count = 0xff;
                                }
                               else
                                {
                                  mqtt_client.publish(device_status_topic, "Flash not possible: output not enabled");
                                }
                            }
                           else // than it must be numerical
                            {
                              end_dim_value[i] = atoi (mesg); // dit gaat toevallig ook goed als het commando "dim 30" is :-)
                              update_rotary (i);
                            }
                        }
                    }
                }

        //Serial.print("it's a command for node: ");
        //Serial.println(device_dim_1[i]);
        //Serial.print("dim vaue: ");
        //Serial.println(end_dim_value[i],DEC);
              break;
            }
        }
      if (i == process_config.nr_active_channels) // het was geen dim commando
        { if (!(strcmp(channel, device_config_topic))) handle_config(mesg);
        }
    }
}

void flash_a_light()
{
  if (flash_count == 0xff) // first time
    {
      smooth_dim_ticker.attach_ms(process_config.flash_speed, smooth_dim_ticker_interrupt);
      smooth_dim_ticker_status = 1;
      flash_pin = process_config.channel[flash_requested - 1].output_pin;
      flash_count = process_config.flash_duration;
    }
   else
    { if (smooth_dim_ticker_tick)
        { smooth_dim_ticker_tick = 0;
          if (flash_count == 0) // last time
            {
              // restore original value
              analogWrite(flash_pin, process_config.channel[flash_requested - 1].invert_value ? process_config.max_range - current_dim_value[flash_requested - 1] : current_dim_value[flash_requested - 1]);
              flash_requested = 0;
              smooth_dim_ticker.detach();
              smooth_dim_ticker_status = 0;
            }
           else
            { if ((flash_count--) % 2) // even/odd
                {
                  analogWrite(flash_pin, process_config.flash_min_val);
                }
               else
                {
                  analogWrite(flash_pin, process_config.flash_max_val);
                }
            }
        }
    }
}

void init_gpio_input_output_table()
{
  memset (gpio_pins, 0, sizeof (gpio_pins)); // nulstellen
}

void set_hostname()
{ char *s = process_config.my_id;
  char *d = device_hostname;

  while (*s)
    {
      if ((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') || (*s >= '0' && *s <= '9'))
        {
          *d = *s;
        }
       else
        {
          *d = '_';
        }

      d++;
      s++;
    }
  *d = 0; // end the string
}

void update_or_init_config(byte current_eeprom_version, byte needed_version, struct configuration *dest)
{ // function tries to use as much as possible from the old config
  // first define the OLD config structure. Pitty is this consumes RAM for a 1-time action..
  struct wifi_nw_v_1
    {
      char ssid [15];
      char pwd [40];
      char ip_mqtt_server[16]; // nieuw
      byte valid;
    };

  struct channel_values_v_1
    {
      byte output_pin;
      byte invert_value;
      byte output_enabled;
      byte rotary_1_pin;
      byte rotary_2_pin;
      byte rotary_value_change_per_pulse;
      byte rotary_enabled;
      byte on_off_switch_pin;
      byte on_off_switch_type; // pulse(0 or 2) or on/off (1 or 3), 0,1 are connected to internal dimmer-value. 2,3 only send 'toggle' command over confirm channel
      byte on_off_switch_enable;
      byte led_indicator_pin;
      byte led_indicator_enabled;
      byte min_value;
      byte max_value;
      byte low_dimmed_value;
    };

  struct configuration_v_1
    { byte config_version;
      struct wifi_nw_v_1 wifi_network[nr_networks]; // first is config network, next is working network
      struct channel_values_v_1 channel[nr_channels_max];
      byte nr_active_channels;
      int dimspeed_command;
      int dimspeed_manual;
      int pwm_frequency;
      byte max_range;
      char my_id[10];
    } old_eeprom_process_config;

// copy config data from EEPROM to new configstructure in RAM
// depending on the changes, this section can be simple or item-by-item

  if (current_eeprom_version == 1 && needed_version == 102)
    {
      int size = sizeof (struct configuration_v_1); // only copy the 'real' old config-data
      char *p = (char *)dest;
      int i;

      for (i=0; i<size; i++) *p++ = EEPROM.read(i);
// new in config 102:
      dest->config_version = needed_version;
      dest->flash_speed = 30;
      dest->flash_duration = 20;
      dest->flash_min_val = 21;
      dest->flash_max_val = 79;

      config_has_been_updated = 1;
    }
   else
    {
      init_default_config(dest);
      config_has_been_updated = 2;
    }
}
