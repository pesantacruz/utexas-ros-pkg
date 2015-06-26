package com.example.pesantacruz.networkalignment;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.ActionBarActivity;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Enumeration;


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

    //IP ArrayList initialization
    ArrayList<String> ips = new ArrayList<String>();

    //UI Elements
    public EditText target_port_text;
    public EditText my_port_text;
    public TextView displayRecMessage;
    public Spinner message_spinner;
    public LoggingManager loggingManager;
    Handler updateConversationHandler;
    public TextView displaySearchMessage;
    Button neighbors;
    Spinner spinner;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

//        this.serverThread = new Thread(new ServerThread());
//        this.serverThread.start();

        target_port_text = (EditText) findViewById(R.id.target_port_text);
        my_port_text = (EditText) findViewById(R.id.my_port_text);
        displayRecMessage = (TextView) findViewById(R.id.texttoscreen);
        displayRecMessage.setMovementMethod(new ScrollingMovementMethod());
        message_spinner = (Spinner) findViewById(R.id.spinner_messages);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.message_options, android.R.layout.simple_spinner_item);

        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        message_spinner.setAdapter(adapter);
        displaySearchMessage = (TextView) findViewById(R.id.searchtext);
        displaySearchMessage.setVisibility(View.GONE);
        displayRecMessage = (TextView) findViewById(R.id.texttoscreen);
        neighbors = (Button) findViewById(R.id.neighbors);
        updateConversationHandler = new Handler();
        loggingManager = new LoggingManager();

        //Add first entry to ArrayList - Spinner Preset Message
        ips.add("Please Select an IP Address...");

        //Retrieve own IP address and add to ArrayList
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements(); ) {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements(); ) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress()) {
                        //Allows for only IPv4 addresses
                        if (inetAddress instanceof Inet4Address) {
                            String ip = inetAddress.getHostAddress().toString();
                            ips.add(ip);
                        }

                    }
                }
            }

        } catch (SocketException obj) {
            Log.e("Error", obj.toString());
        }

        //Create Spinner
        spinner = (Spinner) findViewById(R.id.ip_spinner);
        //Create an ArrayAdapter using a default spinner layout and an Arraylist
        ArrayAdapter myAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, ips) {
            //Allows 'Please Select...' text to appear
            @Override
            public View getDropDownView(int position, View convertView, ViewGroup parent) {
                View v = null;

                //If this is the initial dummy entry, make it hidden
                if (position == 0) {
                    TextView tv = new TextView(getContext());
                    tv.setHeight(0);
                    tv.setVisibility(View.GONE);
                    v = tv;
                } else {
                    //Pass convertView as null to prevent reuse of special case views
                    v = super.getDropDownView(position, null, parent);
                }

                //Hide scroll bar because it appears sometimes unnecessarily, this does not prevent scrolling
                parent.setVerticalScrollBarEnabled(false);
                return v;
            }
        };
        //Specify layout to use when list of choices appears
        myAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        //Apply adapter to spinner
        spinner.setAdapter(myAdapter);

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
        outState.putString("target_port_text", target_port_text.getText().toString());
        outState.putString("my_port_text", my_port_text.getText().toString());
        outState.putString("displayRecMessage", displayRecMessage.getText().toString());
    }

    @Override
    protected void onRestoreInstanceState(Bundle savedState) {
        super.onRestoreInstanceState(savedState);
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
            TARGET_SERVERPORT = Integer.parseInt(target_port_text.getText().toString());
            new Thread(new ClientThread()).start();
        } catch (Exception e) {
            //e.printStackTrace();
        }
    }

    //Searching for Neighbors Button Press
    public void onClick_Search(View view) {
        //Displays Searching... message
        displaySearchMessage.setVisibility(View.VISIBLE);

        Thread searchThread = new Thread(new SearchThread());
        searchThread.start();
    }

    public void clearView() {
        displayRecMessage.setText("");
    }

    public void viewLogs() {
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
            } catch (IOException e) {
                e.printStackTrace();
            }

            while (!Thread.currentThread().isInterrupted()) {
                try {
                    System.out.println("Trying to Accept...");
                    Socket incomingSocket = serverSocket.accept();    // This will block until there is an incoming socket

                    Thread communicationThread = new Thread(new CommunicationThread(incomingSocket));
                    communicationThread.start();

                } catch (IOException e) {
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

        public CommunicationThread(Socket incomingSocket) {
            this.incomingSocket = incomingSocket;

            try {
                this.data = handleClient(this.incomingSocket);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public byte[] handleClient(Socket incomingSocket) throws IOException {
            message = "";
            Decoder decoder = new Decoder(incomingSocket, loggingManager);
            message += decoder.message;

            System.out.println("Message Received..." + message);

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

        public UpdateUIThread(String message) {
            this.recDataString = message;
        }

        @Override
        public void run() {
            displayRecMessage.setText(displayRecMessage.getText().toString() + "Client Says: " + recDataString + "\n");
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
                sendMessage(clientSocket);
                System.out.println("Message Sent...");
                clientSocket.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        }


        public void sendMessage(Socket clientSocket) throws IOException {
            Spinner mySpinner = (Spinner) findViewById(R.id.spinner_messages);
            String message = mySpinner.getSelectedItem().toString();
            byte[] source = new byte[2];
            byte[] s = clientSocket.getInetAddress().getAddress();
            source[0] = s[2];
            source[1] = s[3];
            byte[] destination = new byte[2];
            byte[] d = clientSocket.getLocalAddress().getAddress();
            destination[0] = d[2];
            destination[1] = d[3];

            Framer.frameMsg(clientSocket, message, source, destination, 0);
            loggingManager.sendTo(clientSocket.getInetAddress().getHostAddress(), message.length());

        }
    }

    //SearchThread//
    class SearchThread implements Runnable {

        public void run() {
            //Initialize InetAddress
            InetAddress inetAddress = null;
            String ip = "169.87.149.";
            //Checks 169.87.149.1 - 169.87.149.254
            for (int i = 165; i < 169; i++) {
                try {
                    inetAddress = InetAddress.getByName(ip + String.valueOf(i));
                    System.out.println("Trying: " + ip + i);
                    //Add IP to ArrayList if reachable and not already present
                    if (inetAddress.isReachable(1000) && (!ips.contains(inetAddress.getHostName()))) {
                        ips.add(inetAddress.getHostName());
                    }
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                @Override
                public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
                    //Retrieves selected IP
                    adapterView.getItemAtPosition(i);
                    TARGET_SERVER_IP = (String) adapterView.getItemAtPosition(i);
                    //Hides Searching... message
                    displaySearchMessage.setVisibility(View.GONE);
                }

                @Override
                public void onNothingSelected(AdapterView<?> adapterView) {

                }
            });
        }
    }

}
