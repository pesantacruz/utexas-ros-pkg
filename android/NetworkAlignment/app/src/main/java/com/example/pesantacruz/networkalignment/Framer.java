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
public class Framer {
    //length
    //final destination
    //original source
    //hop count
    //message
    public static void frameMsg(Socket clientSocket, String message,byte[] source, byte[] destination, int count)throws IOException{
        InputStream in = clientSocket.getInputStream();
        OutputStream out = clientSocket.getOutputStream();
        DataInputStream dis = new DataInputStream(in);
        DataOutputStream dos = new DataOutputStream(out);

        dos.writeInt(message.length());
        dos.write(destination);
        dos.write(source);
        dos.writeInt(count);
        dos.writeBytes(message);
        dos.flush();
    }

    public void frameMsg(Socket clientSocket, byte[] message,String source, String destination, int count)throws IOException{
        InputStream in = clientSocket.getInputStream();
        OutputStream out = clientSocket.getOutputStream();
        DataInputStream dis = new DataInputStream(in);
        DataOutputStream dos = new DataOutputStream(out);

        dos.writeInt(message.length);
        dos.writeBytes(destination);
        dos.writeBytes(source);
        dos.writeInt(count);
        dos.write(message);
        dos.flush();
    }
}
