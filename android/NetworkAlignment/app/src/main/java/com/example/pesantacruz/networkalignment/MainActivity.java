package com.example.pesantacruz.networkalignment;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.ActionBarActivity;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;


public class MainActivity extends ActionBarActivity {
    //Socket initialization
    private ServerSocket serverSocket;
    //private static final int SERVERPORT = 6000;
    //private static final String TARGET_SERVER_IP = "169.87.149.135";
    //private static final int TARGET_SERVERPORT = 6001;

    public int SERVERPORT;
    public int TARGET_SERVERPORT;
    public String TARGET_SERVER_IP;


    //Thread initialization
    Thread serverThread = null;

    //UI Elements
    public EditText target_ip_text;
    public EditText target_port_text;
    public EditText my_port_text;
    public TextView displayRecMessage;
    Handler updateConversationHandler;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

//        this.serverThread = new Thread(new ServerThread());
//        this.serverThread.start();

        target_ip_text = (EditText) findViewById(R.id.target_ip_text);
        target_port_text = (EditText) findViewById(R.id.target_port_text);
        my_port_text = (EditText) findViewById(R.id.my_port_text);
        displayRecMessage = (TextView) findViewById(R.id.texttoscreen);

        updateConversationHandler = new Handler();

        System.out.println("Server initialized...");


    }

    //Start Server Button
    public void onClick_Start(View view) {
        SERVERPORT = Integer.parseInt(my_port_text.getText().toString());

        this.serverThread = new Thread(new ServerThread());
        this.serverThread.start();
    }


    //Button Press
    public void onClick_Send(View view) {
        System.out.println("Button Pressed...");

        TARGET_SERVER_IP = target_ip_text.getText().toString();
        TARGET_SERVERPORT = Integer.parseInt(target_port_text.getText().toString());

        new Thread(new ClientThread()).start();
    }


    //ServerThread//
    class ServerThread implements Runnable {

        @Override
        public void run() {
            try {
                System.out.println("Creating Server Socket...");
                serverSocket = new ServerSocket(SERVERPORT);
            }
            catch (IOException e){
                e.printStackTrace();
            }

            while (!Thread.currentThread().isInterrupted()){
                try{
                    System.out.println("Trying to Accept...");
                    Socket incomingSocket = serverSocket.accept();    // This will block until there is an incoming socket

                    Thread communicationThread = new Thread(new CommunicationThread(incomingSocket));
                    communicationThread.start();

                }
                catch (IOException e){
                    e.printStackTrace();
                }
            }
        }
    }


    //CommunicationThread//
    class CommunicationThread implements Runnable {
        public Socket incomingSocket;
        private byte[] data;

        public CommunicationThread(Socket incomingSocket){
            this.incomingSocket = incomingSocket;

            try {
                this.data = handleClient(this.incomingSocket);
            }
            catch (IOException e){
                e.printStackTrace();
            }
        }

        public byte[] handleClient(Socket incomingSocket) throws IOException{
            InputStream in = incomingSocket.getInputStream();
            OutputStream out = incomingSocket.getOutputStream();

            DataInputStream dis = new DataInputStream(in);
            DataOutputStream dos = new DataOutputStream(out);

            int len = dis.readInt();
            byte[] data = new byte[len];

            if (len>0) {
                dis.readFully(data);
            }

            System.out.println("Data Received..." + data);

            return data;

        }

        public void run() {
           // while (!Thread.currentThread().isInterrupted()) {
           //     System.out.println("Entering Comm Thread...");
                updateConversationHandler.post(new UpdateUIThread(data));
           // }
        }
    }


    //UpdateUIThread
    class UpdateUIThread implements Runnable {
        private String recDataString;

        public UpdateUIThread(byte[] data) {
            this.recDataString = data.toString();
        }

        @Override
        public void run() {
            displayRecMessage.setText(displayRecMessage.getText().toString()+"Client Says: "+ recDataString + "\n");
        }
    }

    class ClientThread implements Runnable {
        byte[] myByteArray = new byte[]{127, -127, 0};

        @Override
        public void run() {
            try {
                InetAddress targetServerAddr = InetAddress.getByName(TARGET_SERVER_IP);

                Socket clientSocket = new Socket(targetServerAddr, TARGET_SERVERPORT);

                System.out.println("Client Socket created...");


                sendBytes(clientSocket, myByteArray);

                System.out.println("Message Sent...");


                clientSocket.close();
            }
            catch (IOException e1) {
                e1.printStackTrace();
            }
        }

        public void sendBytes(Socket clientSocket, byte[] myByteArray) throws IOException {
            sendBytes(clientSocket, myByteArray, 0, myByteArray.length);
        }

        public void sendBytes(Socket clientSocket, byte[] myByteArray, int start, int len) throws IOException {
            InputStream in = clientSocket.getInputStream();
            OutputStream out = clientSocket.getOutputStream();

            DataInputStream dis = new DataInputStream(in);
            DataOutputStream dos = new DataOutputStream(out);

            dos.writeInt(len);
            if (len >0) {
                dos.write(myByteArray, start, len);
            }
        }
    }

}
