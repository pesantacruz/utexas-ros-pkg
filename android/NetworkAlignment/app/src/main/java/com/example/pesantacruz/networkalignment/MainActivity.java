package com.example.pesantacruz.networkalignment;

import android.app.Activity;
import android.content.Intent;
import android.content.res.Resources;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.ActionBarActivity;
import android.text.method.ScrollingMovementMethod;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;
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
    public Spinner message_spinner;
    public LoggingManager loggingManager;
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
        displayRecMessage.setMovementMethod(new ScrollingMovementMethod());
        message_spinner = (Spinner) findViewById(R.id.spinner_messages);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.message_options, android.R.layout.simple_spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        message_spinner.setAdapter(adapter);
        updateConversationHandler = new Handler();
        loggingManager = new LoggingManager();

        System.out.println("Server initialized...");


    }
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main_activity_menu, menu);
        return super.onCreateOptionsMenu(menu);
    }
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle item selection
        switch (item.getItemId()) {
            case R.id.clear_view:
                clearView();
                return true;
            case R.id.view_logs:
                viewLogs();
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }
    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putString("target_ip_text", target_ip_text.getText().toString());
        outState.putString("target_port_text", target_port_text.getText().toString());
        outState.putString("my_port_text", my_port_text.getText().toString());
        outState.putString("displayRecMessage", displayRecMessage.getText().toString());
    }

    @Override
    protected void onRestoreInstanceState(Bundle savedState) {
        super.onRestoreInstanceState(savedState);
        target_ip_text.setText(savedState.getString("target_ip_text"));
        target_port_text.setText(savedState.getString("target_port_text"));
        my_port_text.setText(savedState.getString("my_port_text"));
        displayRecMessage.setText(savedState.getString("displayRecMessage"));
    }

    //Start Server Button
    public void onClick_Start(View view) {
        try {
            SERVERPORT = Integer.parseInt(my_port_text.getText().toString());
            this.serverThread = new Thread(new ServerThread());
            this.serverThread.start();
        } catch (Exception e) {
            //e.printStackTrace();
        }
    }


    //Button Press
    public void onClick_Send(View view) {
        try {
            System.out.println("Button Pressed...");
            TARGET_SERVER_IP = target_ip_text.getText().toString();
            TARGET_SERVERPORT = Integer.parseInt(target_port_text.getText().toString());
            new Thread(new ClientThread()).start();
        } catch (Exception e) {
            //e.printStackTrace();
        }
    }

    public void clearView() {
        displayRecMessage.setText("");
    }
    public void viewLogs(){
        Intent intent = new Intent(this, ViewLogActivity.class);
        intent.putExtra("log", loggingManager.getLog());
        startActivity(intent);
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
        private String message;

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
            message = "";
            while(!incomingSocket.isClosed()) {
                try {
                    int len = dis.readInt();
                    byte[] data = new byte[len];

                    if (len>0) {
                        dis.readFully(data);
                    }
                    loggingManager.receiveFrom(incomingSocket.getInetAddress().getHostAddress(), len);
                    String input = new String(data);
                    message += input;
                    System.out.println("Data Received..." + data);
                    System.out.println("Message Received..." + input);
                    String[] message_array = getResources().getStringArray(R.array.message_options);
                    boolean isValid = false;
                    for (String s : message_array) {
                        if (s.equals(input)) {
                            isValid = true;
                        }
                    }
                    if (isValid) {
                        System.out.println("Message is Valid.");
                    } else {
                        System.out.println("Message is Invalid.");
                    }
                    message += isValid + "\n";
                } catch (IOException e) {
                    break;
                }
            }
            return data;

        }

        public void run() {
           // while (!Thread.currentThread().isInterrupted()) {
           //     System.out.println("Entering Comm Thread...");
                updateConversationHandler.post(new UpdateUIThread(message));
           // }
        }
    }


    //UpdateUIThread
    class UpdateUIThread implements Runnable {
        private String recDataString;

        public UpdateUIThread(byte[] data) {
            this.recDataString = data.toString();
        }
        public UpdateUIThread(String message){this.recDataString = message;}

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


                //sendBytes(clientSocket, myByteArray);
                //for (int i = 0; i < 100; i++) {
                    sendMessage(clientSocket);
                //}
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
            String message = "Hello World";
            len = message.length();
            myByteArray = message.getBytes();
            dos.writeInt(len);
            if (len >0) {
                dos.write(myByteArray, start, len);
            }
        }

        public void sendMessage(Socket clientSocket) throws IOException {
            InputStream in = clientSocket.getInputStream();
            OutputStream out = clientSocket.getOutputStream();
            DataInputStream dis = new DataInputStream(in);
            DataOutputStream dos = new DataOutputStream(out);
            Spinner mySpinner=(Spinner) findViewById(R.id.spinner_messages);
            String message = mySpinner.getSelectedItem().toString();
            dos.writeInt(message.length());
            dos.writeBytes(message);
            loggingManager.sendTo(clientSocket.getInetAddress().getHostAddress(), message.length());

        }
    }

}
