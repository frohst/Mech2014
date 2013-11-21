
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(9,10); //SPI off of ce pin 9&cs pin10

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };  //some shit off the interwebs (ssoi)
typedef enum { role_ping_out = 1, role_pong_back } role_e;
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

role_e role = role_pong_back;    //the job

void setup()
{
  Serial.begin(57600);
  printf_begin();
  printf("\n\rf24/examples/GettingStarted/\n\r");
  printf("ROLE: %s\n\r",role_friendly_name[role]);
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");
  printf("*** PRESS 'R' to begin recieving information from the other node\n\r");
  
  radio.begin();
  radio.setRetries(15,15);    //delay
  
  radio.setPayloadSize(8);
  
  if (role==role_ping_out)  //sets pipes based on role
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPip(1,pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }
  
  radio.startListening();
  radio.printDetails();  //TELL ME YOUR SECRETS OH WISE CHIP
}

void loop()
{
  if (role == role_ping_out)
  {
    radio.stopListening();  //can't talk and listen at same time
    unsigned long time = millis();
    printf("Now sending %lu...",time);
    bool ok = radio.write( &time, sizeof(unsigned long) );
    
    if(ok)
    {
      printf("ok...");
    }
    else
    {
      printf("failed.\n\r");
    }
    
    radio.startListening();  //said what needed to be said. now i shall wait
    
    unsigned long started_watiing_at = millis();
    bool timeout = false;
    while(!radio.available() && !timeout)
    {
      if (millis() - started_waiting_at > 250)
      {
        timeout = true;
      }
    }
    
    if (timeout)
    {
      printf("Failed, response timed out.\n\r");
    }
    else
    {
      unsigned long got_time;
      radio.read(&got_time, sizeof(unsigned long));
      
      printf("Got response %lu, rount-trip delay: %lu\n\r", got_time,millis()-got_time);
    }
    delay(1000);
  }
  
  if ( role == role_pong_back )
  {
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &got_time, sizeof(unsigned long) );

        // Spew it
        printf("Got payload %lu...",got_time);

	// Delay just a little bit to let the other unit
	// make the transition to receiver
	delay(20);
      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &got_time, sizeof(unsigned long) );
      printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
  }

  //
  // Change roles
  //

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == role_pong_back )
    {
      printf("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK\n\r");

      // Become the primary transmitter (ping out)
      role = role_ping_out;
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }
    else if ( c == 'R' && role == role_ping_out )
    {
      printf("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK\n\r");
      
      // Become the primary receiver (pong back)
      role = role_pong_back;
      radio.openWritingPipe(pipes[1]);
      radio.openReadingPipe(1,pipes[0]);
    }
  }
}
