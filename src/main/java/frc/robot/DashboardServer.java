package frc.robot;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.json.JSONObject;

import io.javalin.Javalin;
import io.javalin.websocket.WsConnectContext;
import io.javalin.websocket.WsContext;

public class DashboardServer {
    Javalin app;
    Set<WsContext> clients = Collections.synchronizedSet(new HashSet<>());
    Map<String, Object> parameters = new HashMap<>();

    enum EventType {
        PARAMETER_UPDATE
    }

    enum ValueType {
        DOUBLE
    }

    public DashboardServer() {
        app = Javalin.create().start(5805);

        app.ws("/dashboard", ws -> {
            ws.onConnect(ctx -> {
                clients.add(ctx);
            });

            ws.onMessage(msg -> {
                JSONObject object = new JSONObject(msg);
                String eventString = object.getString("event");
                EventType event;
                try {
                    event = EventType.valueOf(eventString);
                } catch (Exception err) {
                    // Throw error
                    return;
                }

                switch (event) {
                    case PARAMETER_UPDATE: {
                        String name = object.getString("name");
                        ValueType valueType = ValueType.DOUBLE;
                        try {
                            String type = object.getString("value_type");
                            valueType = ValueType.valueOf(type);
                        } catch (Exception err) {
                            return;
                        }

                        switch (valueType) {
                            case DOUBLE: {
                                double value = object.getDouble("value");
                                parameters.put(name, value);
                                break;
                            }
                        }

                        break;
                    } 
                }
            });

            ws.onClose(ctx -> {
                clients.remove(ctx);
            });
        });
    }

    public double getDouble(String name, double defaultValue) {
        if (parameters.containsKey(name)) {
            return (double)parameters.get(name);
        }
        return defaultValue;
    }

    public void broadcastMessage(String message) {
        for (var client : clients) {
            client.send(message);
        }
    }
}
