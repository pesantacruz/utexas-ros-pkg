package com.example.pesantacruz.networkalignment;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

/**
 * Created by Michael on 6/22/2015.
 */
public class Decoder {
    int length;
    byte[] destination;
    byte[] source;
    int count;
    String message;
    public static final int NAMELENGTH = 2;

    public Decoder(Socket incomingSocket, LoggingManager loggingManager) throws IOException{
        InputStream in = incomingSocket.getInputStream();
        OutputStream out = incomingSocket.getOutputStream();

        DataInputStream dis = new DataInputStream(in);
        DataOutputStream dos = new DataOutputStream(out);
        message = "";
        while(!incomingSocket.isClosed()) {
            try {
                length = dis.readInt();
                byte[] data = new byte[length];
                byte[] des = new byte[NAMELENGTH];
                byte[] sor = new byte[NAMELENGTH];
                dis.readFully(des);
                dis.readFully(sor);
                count = dis.readInt();
                if (length>0) {
                    dis.readFully(data);
                }
                String input = new String(data);
                message += input +"\n";
                loggingManager.receiveFrom(incomingSocket.getInetAddress().getHostAddress(), length);

            } catch (IOException e) {
                break;
            }
        }
    }
}
