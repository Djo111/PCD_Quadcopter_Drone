    #include <WiFi.h>
 
   
const char* ssid = "Mi 9 Lite";
const char* password = "12345678@";


WiFiServer server(80);

double Kp = 0.0;
double Kd = 0.0;
double Ki = 0.0;
char str[20],str2[20];
int16_t p;

void setupwificalib(){
//      if (!SPIFFS.begin()) {
//     Serial.println("Failed to mount file system");
//     return;
//   }
//   file = SPIFFS.open("/text.txt",FILE_WRITE);

Serial.begin(9600);
WiFi.begin(ssid, password);
// digitalWrite(16,0);
while (WiFi.status() != WL_CONNECTED)
{
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  //digitalWrite(16,1);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
}



void mainwificalib(double& kp, double& ki, double& kd , int16_t pitch,int16_t roll){
     
//   if (!file) {
//     Serial.println("Failed to open file for writing");
//     return;
//   }
//     WiFiClient client = server.available();
//   if (client) {
//     Serial.println("New client connected");
//     String request = client.readStringUntil('\r');
//     //Serial.println(request);
//     client.flush();

//     if (request.indexOf("/kp+") != -1) {
//       Kp += 0.01;
//     }
//     else if (request.indexOf("/kp-") != -1) {
//       Kp -= 0.01;
//     }
//    else if (request.indexOf("/kpm+") != -1) {
//       Kp += 0.1;
//     }
//     else if (request.indexOf("/kpm-") != -1) {
//       Kp -= 0.1;
//     }
//     else if (request.indexOf("/kpmm") != -1) {
//       Kp += 1.;
//     }
//     else if (request.indexOf("/kpmn") != -1) {
//       Kp -= 1.;
//     }
//     else if (request.indexOf("/kd+") != -1) {
//       Kd += 0.1;
//     }
//     else if (request.indexOf("/kd-") != -1) {
//       Kd -= 0.1;
//     }
//       else if (request.indexOf("/kdm+") != -1) {
//       Kd += 1.;
//     }
//     else if (request.indexOf("/kdm-") != -1) {
//       Kd -= 1.;
//     }
//     else if (request.indexOf("/ki+") != -1) {
//       Ki += 0.01;
//     }
//     else if (request.indexOf("/ki-") != -1) {
//       Ki -= 0.01;
//     }

//     client.println("HTTP/1.1 200 OK");
//     client.println("Content-Type: text/html");
//     client.println();
//     client.println("<!DOCTYPE html>");
//     client.println("<html>");
//     client.println("<head><title>ESP32 Increment by Button Example</title></head>");
//     client.println("<body>");
//     client.print("<h1>Kp:</h1><p>Number: ");
//     client.print(Kp);
//     client.println("</p><a href=\"/kp+\"><button>+0.01</button></a>");
//     client.println("<a href=\"/kp-\"><button>-0.01</button></a><br>");
//     client.println("</p><a href=\"/kpm+\"><button>+0.1</button></a>");
//     client.println("<a href=\"/kpm-\"><button>-0.1</button></a><br>");
//     client.println("</p><a href=\"/kpmm\"><button>+1</button></a>");
//     client.println("<a href=\"/kpmn\"><button>-1</button></a><br>");
//     client.print("<h1>Kd:</h1><p>Number: ");
//     client.print(Kd);
//     client.println("</p><a href=\"/kd+\"><button>+0.1</button></a>");
//     client.println("<a href=\"/kd-\"><button>-0.1</button></a><br>");
//     client.println("</p><a href=\"/kdm+\"><button>+1</button></a>");
//     client.println("<a href=\"/kdm-\"><button>-1</button></a><br>");
//     client.print("<h1>Ki:</h1><p>Number: ");
//     client.print(Ki);
//     client.println("</p><a href=\"/ki+\"><button>+</button></a>");
//     client.println("<a href=\"/ki-\"><button>-</button></a><br>");
//     client.println("</body></html>");
    
  
    
//     client.stop();
    
    
//     // Serial.print("kp:");
//     // Serial.print(Kp);
//     // Serial.print(" kd:");
//     // Serial.print(Kd);
//     // Serial.print(" ki:");
//     // Serial.print(Ki);
//     // Serial.println("Client disconnected");
//   }
 // Check if a client has connected
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");

    // Wait for some data to arrive
   

    // Read the data
    String data = client.readStringUntil('\r');
    Serial.println("Received data: " + data);

    // Parse the data
    if (data.indexOf("kp_inc0") != -1) {
      Kp += 0.01;
    } else if (data.indexOf("kp_dec0") != -1) {
      Kp -= 0.01;
    } else if (data.indexOf("kp_inc1") != -1) {
      Kp += 0.1;
    } else if (data.indexOf("kp_dec1") != -1) {
      Kp -= 0.1;
    }else if (data.indexOf("kp_inc2") != -1) {
      Kp += 1;
    } else if (data.indexOf("kp_dec2") != -1) {
      Kp -= 1;
    }

    else if (data.indexOf("kd_inc0") != -1) {
      Kd += 0.001;
    } else if (data.indexOf("kd_dec0") != -1) {
      Kd -= 0.001;
    } else if (data.indexOf("kd_inc1") != -1) {
      Kd += 0.01;
    } else if (data.indexOf("kd_dec1") != -1) {
      Kd -= 0.01;
    } else if (data.indexOf("kd_inc2") != -1) {
      Kd += 1;
    }else if (data.indexOf("kd_dec2") != -1) {
      Kd -= 1;
    }
    
     else if (data.indexOf("ki_inc0") != -1) {
      Ki += 0.0001;
    }else if (data.indexOf("ki_dec0") != -1) {
      Ki -= 0.0001;
    } else if (data.indexOf("ki_inc1") != -1) {
      Ki += 0.001;
    }else if (data.indexOf("ki_dec1") != -1) {
      Ki -= 0.001;
    }else if (data.indexOf("ki_inc2") != -1) {
      Ki += 0.01;
    }else if (data.indexOf("ki_dec2") != -1) {
      Ki -= 0.01;
    }

//         if (data.indexOf("/kp+") != -1) {
//       Kp += 0.01;
//     }
//     else if (data.indexOf("/kp-") != -1) {
//       Kp -= 0.01;
//     }
//    else if (data.indexOf("/kpm+") != -1) {
//       Kp += 0.1;
//     }
//     else if (data.indexOf("/kpm-") != -1) {
//       Kp -= 0.1;
//     }
//     else if (data.indexOf("/kpmm") != -1) {
//       Kp += 1.;
//     }
//     else if (data.indexOf("/kpmn") != -1) {
//       Kp -= 1.;
//     }
//     else if (data.indexOf("/kd+") != -1) {
//       Kd += 0.1;
//     }
//     else if (data.indexOf("/kd-") != -1) {
//       Kd -= 0.1;
//     }
//       else if (data.indexOf("/kdm+") != -1) {
//       Kd += 1.;
//     }
//     else if (data.indexOf("/kdm-") != -1) {
//       Kd -= 1.;
//     }
//     else if (data.indexOf("/ki+") != -1) {
//       Ki += 0.01;
//     }
//     else if (data.indexOf("/ki-") != -1) {
//       Ki -= 0.01;
//     }

    // Send a response
    // Send a response
    p = pitch;
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("");
    client.println("<html><body>");
    client.println("<h1>Variables updated:</h1>");
    client.println("<form>");
    client.print("<p>kp = "); client.print(Kp); client.println("</p>");
    client.print("<button name=\"kp_inc0\" value=\"true\">+0.01</button>");
    client.print("<button name=\"kp_dec0\" value=\"true\">-0.01</button>");
    client.println("<br>");
    client.print("<button name=\"kp_inc1\" value=\"true\">+0.1</button>");
    client.print("<button name=\"kp_dec1\" value=\"true\">-0.1</button>");
    client.println("<br>");
    client.print("<button name=\"kp_inc2\" value=\"true\">+1</button>");
    client.print("<button name=\"kp_dec2\" value=\"true\">-1</button>");
    client.println("<br>");
    dtostrf(Kd, 1, 3, str);
    client.print("<p>kd = "); client.print(str); client.println("</p>");
    client.print("<button name=\"kd_inc0\" value=\"true\">+0.001</button>");
    client.print("<button name=\"kd_dec0\" value=\"true\">-0.001</button>");
    client.println("<br>");
    client.print("<button name=\"kd_inc1\" value=\"true\">+0.01</button>");
    client.print("<button name=\"kd_dec1\" value=\"true\">-0.01</button>");
    client.println("<br>");
    client.print("<button name=\"kd_inc2\" value=\"true\">+1</button>");
    client.print("<button name=\"kd_dec2\" value=\"true\">-1</button>");
    client.println("<br>");
    dtostrf(Ki, 1, 4, str2);
    client.print("<p>ki = "); client.print(str2); client.println("</p>");
    client.print("<button name=\"ki_inc0\" value=\"true\">+0.0001</button>");
    client.print("<button name=\"ki_dec0\" value=\"true\">-0.0001</button>");
    client.println("<br>");
    client.print("<button name=\"ki_inc1\" value=\"true\">+0.001</button>");
    client.print("<button name=\"ki_dec1\" value=\"true\">-0.001</button>");
    client.println("<br>");
    client.print("<button name=\"ki_inc2\" value=\"true\">+0.01</button>");
    client.print("<button name=\"ki_dec2\" value=\"true\">-0.01</button>");
    client.println("<br>");
    client.print("<h1>Pitch: </h1>");
    client.print(pitch);
    client.println("<br>");
    client.print("<h1>Roll: </h1>");
    client.print(roll);
    // client.print("<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>");
    // client.print("<script>var chartData = {labels:[], datasets:[{label:'Sensor Value',data:[]}],};");
    // client.print("var chartOptions = {responsive:true,scales:{yAxes:[{ticks:{beginAtZero:true}}]},};");
    // client.print("var lineChart = new Chart(document.getElementById('chart').getContext('2d'),{type:'line',data:chartData,options:chartOptions});");
    // client.print("setInterval(function() {updateChart();}, 1000);");
    // client.print("function updateChart() {chartData.labels.push(new Date().toLocaleTimeString());chartData.datasets[0].data.push(" + String(pitch) + ");lineChart.update();}");
    // client.print("</script></head><body><h1>Real-time Chart</h1><canvas id=\"chart\" width=\"800\" height=\"400\"></canvas></body></html>");

    client.println("</form>");
    client.println("</body></html>");

    // Disconnect the client
    client.stop();
    Serial.println("Client disconnected");
    kp = Kp;
    kd = Kd;
    ki = Ki;
    
  }
}