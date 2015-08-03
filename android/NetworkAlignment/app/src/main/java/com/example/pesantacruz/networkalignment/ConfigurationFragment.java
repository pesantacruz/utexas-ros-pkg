package com.example.pesantacruz.networkalignment;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.LayoutInflater;
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
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Enumeration;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 *
 * to handle interaction events.
 * Use the {@link ConfigurationFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class ConfigurationFragment extends Fragment {
    //Socket initialization
    private ServerSocket serverSocket;
    //private static final int SERVERPORT = 6000;
    //private static final String TARGET_SERVER_IP = "169.87.149.135";
    //private static final int TARGET_SERVERPORT = 6001;

    public int SERVERPORT;
    public int TARGET_SERVERPORT;
    public String SENDING_IP;
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
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;


    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     * @return A new instance of fragment ConfigurationFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static ConfigurationFragment newInstance() {
        ConfigurationFragment fragment = new ConfigurationFragment();
        return fragment;
    }

    public ConfigurationFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setRetainInstance(true);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        super.onCreate(savedInstanceState);
        View view = inflater.inflate(R.layout.fragment_configuration, container, false);

//        this.serverThread = new Thread(new ServerThread());
//        this.serverThread.start();

        target_port_text = (EditText) view.findViewById(R.id.target_port_text);
        my_port_text = (EditText) view.findViewById(R.id.my_port_text);
        displayRecMessage = (TextView) view.findViewById(R.id.texttoscreen);
        displayRecMessage.setMovementMethod(new ScrollingMovementMethod());
        message_spinner = (Spinner) view.findViewById(R.id.spinner_messages);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this.getActivity(),
                R.array.message_options, android.R.layout.simple_spinner_item);

        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        message_spinner.setAdapter(adapter);
        displaySearchMessage = (TextView) view.findViewById(R.id.searchtext);
        displaySearchMessage.setVisibility(View.GONE);
        displayRecMessage = (TextView) view.findViewById(R.id.texttoscreen);
        //displayRecMessage.setVisibility(View.GONE);
        neighbors = (Button) view.findViewById(R.id.neighbors);
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
                            String ip = inetAddress.getHostAddress();
                            ips.add(ip);
                        }

                    }
                }
            }

        } catch (SocketException obj) {
            Log.e("Error", obj.toString());
        }

        //Create Multiple Selection Spinner
        MultipleSelectionSpinner.MultiSpinnerListener listener = new MultipleSelectionSpinner.MultiSpinnerListener() {
            @Override
            public void onItemsSelected(boolean[] selected) {

            }
        };
        MultipleSelectionSpinner multiSpinner = (MultipleSelectionSpinner) view.findViewById(R.id.multi_spinner);
        multiSpinner.setItems(ips, "Please Select an IP Address...", listener);

//        //Create Spinner
//        spinner = (Spinner) view.findViewById(R.id.ip_spinner);
//        //Create an ArrayAdapter using a default spinner layout and an Arraylist
//        ArrayAdapter myAdapter = new ArrayAdapter<String>(this.getActivity(), android.R.layout.simple_spinner_item, ips) {
//            //Allows 'Please Select...' text to appear
//            @Override
//            public View getDropDownView(int position, View convertView, ViewGroup parent) {
//                View v = null;
//
//                //If this is the initial dummy entry, make it hidden
//                if (position == 0) {
//                    TextView tv = new TextView(getContext());
//                    tv.setHeight(0);
//                    tv.setVisibility(View.GONE);
//                    v = tv;
//                } else {
//                    //Pass convertView as null to prevent reuse of special case views
//                    v = super.getDropDownView(position, null, parent);
//                }
//
//                //Hide scroll bar because it appears sometimes unnecessarily, this does not prevent scrolling
//                parent.setVerticalScrollBarEnabled(false);
//                return v;
//            }
//        };
//        //Specify layout to use when list of choices appears
//        myAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
//        //Apply adapter to spinner
//        spinner.setAdapter(myAdapter);

        System.out.println("Server initialized...");

        return view;
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

    public void onClick_Multisend(View v){
        try {
            System.out.println("Button Pressed...");
            TARGET_SERVERPORT = Integer.parseInt(target_port_text.getText().toString());
            for(int i = 0; i < 100; i++) {
                new Thread(new ClientThread()).start();
            }
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

                    InetSocketAddress whatever = (InetSocketAddress) incomingSocket.getRemoteSocketAddress();
                    SENDING_IP = whatever.getHostName();

                    System.out.println("Sending IP = " + SENDING_IP);

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
            final MainActivity2 act = (MainActivity2) getActivity();
            act.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    // This code will always run on the UI thread, therefore is safe to modify UI elements.
                    act.lfrag.displayLog.setText(loggingManager.getLog());
                }
            });
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
            final MainActivity2 act = (MainActivity2) getActivity();
            act.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    // This code will always run on the UI thread, therefore is safe to modify UI elements.
                    act.mfrag.displayMessage.setText(displayRecMessage.getText().toString() + "Client " + SENDING_IP + " says: " + recDataString + "\n");
                }
            });
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
            Spinner mySpinner = (Spinner) getView().findViewById(R.id.spinner_messages);
            String message = mySpinner.getSelectedItem().toString();
            byte[] source = new byte[2];
            byte[] s = clientSocket.getInetAddress().getAddress();
            source[0] = s[2];
            source[1] = s[3];
            byte[] destination = new byte[2];
            byte[] d = clientSocket.getLocalAddress().getAddress();
            destination[0] = d[2];
            destination[1] = d[3];

            Framer.frameMsg(clientSocket, message, source, destination, 1);
            loggingManager.sendTo(clientSocket.getInetAddress().getHostAddress(), message.length());
            final MainActivity2 act = (MainActivity2) getActivity();
            act.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    // This code will always run on the UI thread, therefore is safe to modify UI elements.
                    act.lfrag.displayLog.setText(loggingManager.getLog());
                }
            });
        }
    }

    //SearchThread//
    class SearchThread implements Runnable {

        public void run() {
            //Initialize InetAddress
            InetAddress inetAddress = null;
            String ip = "169.87.149.";
            //Checks 169.87.149.1 - 169.87.149.254
            for (int i = 1; i < 255; i++) {
                try {
                    inetAddress = InetAddress.getByName(ip + String.valueOf(i));
                    System.out.println("Trying: " + ip + i);
                    //Add IP to ArrayList if reachable and not already present
                    if (inetAddress.isReachable(1000) && (!ips.contains(inetAddress.toString().substring(1)))) {
                        System.out.println("Found " + ip + i + " as neighbor");
                        ips.add(inetAddress.toString().substring(1));
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