/*
 * FFC Team
 * FITHOU - VN
 * WebServer handling
*/

/*
 * HTTP server
 * Only work when WiFi is connected
 */

#include <HTTP_Method.h>
#include <WebServer.h>

WebServer http_srv(80);

/* display login / admin page */
void httpRoot(void) {
  http_srv.sendHeader("Cache-Control", "no-cache");
  if (http_srv.hasHeader("Cookie") && http_srv.header("Cookie").indexOf(String("PIN=") + config.pin) >= 0) {
    http_srv.send(200, "text/html",
                  "<!DOCTYPE html>"\
                  "<html>"\
                  "<head>"\
                  "  <title>ESP32 DIY Web</title>"\
                  "</head>"\
                  "<body>"\
                  "<h1 style=\"text-align:center;color:red\">ESP32 DIY Web</h1>"\
                  "<form action=\"/login\" method=\"GET\" style=\"width:100px;margin:0 auto\">"\
                  "  <input style=\"display:block;width:80px;margin:0 auto\" name=\"PIN\" type=\"text\" placeholder=\"4 digits\" pattern=\"[0-9]{4}\"/>"\
                  "  <div style=\"padding-bottom:10px\"></div>"\
                  "  <button style=\"display:block;width:40px;margin:0 auto\" type=\"submit\">Go</button>"\
                  "</form>"\
                  "</body>"\
                  "</html>");
  } else {

  }
}

/* handle /login */
void httpLogin(void) {
  http_srv.sendHeader("Cache-Control", "no-cache");
  if (http_srv.hasArg("PIN") && http_srv.arg("PIN") == config.pin) {
    http_srv.sendHeader("Location", "/");
    http_srv.sendHeader("Cookie", String("PIN=") + config.pin);
  }
  http_srv.sendHeader("Location", "/");
  http_srv.send(307);
}

/* display not found page */
void httpNotFound(void) {
  http_srv.sendHeader("Cache-Control", "no-cache");
  http_srv.send(404, "text/html",
                "<!DOCTYPE html>"\
                "<html>"\
                "<head>"\
                "  <title>404 Not Found</title>"\
                "</head>"\
                "<body>"\
                "<h1 style=\"text-align:center;color:red\">404 Not Found</h1>"\
                "</body>"\
                "</html>");
}

void setup_http_srv(void) {
  Serial.print("HTTP server: starting...");
  http_srv.on("/", httpRoot);
  http_srv.on("/login", httpLogin);
  http_srv.onNotFound(httpNotFound);
  http_srv.begin();
  Serial.println(" OK");
}

void loop_http_srv() {
	http_srv.handleClient();
}

