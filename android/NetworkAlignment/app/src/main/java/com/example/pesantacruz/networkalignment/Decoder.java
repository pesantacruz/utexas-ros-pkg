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
    String destination;
    String source;
    int count;
    byte[] message;
    public static final int NAMELENGTH = 5;

    public Decoder(Socket incomingSocket) throws IOException{
        InputStream in = incomingSocket.getInputStream();
        OutputStream out = incomingSocket.getOutputStream();

        DataInputStream dis = new DataInputStream(in);
        DataOutputStream dos = new DataOutputStream(out);
        while(!incomingSocket.isClosed()) {
            try {
                length = dis.readInt();
                byte[] data = new byte[length];
                byte[] des = new byte[NAMELENGTH];
                byte[] sor = new byte[NAMELENGTH];
                dis.readFully(des);
                dis.readFully(sor);
                destination = new String(des);
                source = new String(sor);
                if (length>0) {
                    dis.readFully(data);
                }

            } catch (IOException e) {
                break;
            }
        }
    }
}
