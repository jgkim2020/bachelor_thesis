using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System;
using System.Linq;

public class ControllerMove : MonoBehaviour {
    public float moveSpeed = 3.0f;
    public float turnSpeed = 360.0f;

    public bool isMoveState = false;

    CharacterController Controller;
    //GvrReticlePointer Gaze; // 1111
    public Camera m_camera; // 1111

    static TcpClient client;
    static byte[] buffer;
    string strdata = "";
    bool isStarted = false;
    const string VELOCITY_KEY = "19775824";
    const string ROTATION_KEY = "38241913";
    const string GAZE_KEY = "22353818";
    // const string ip_address = "192.168.1.34"; // cmd > ipconfig (WLAN IPV4 address) - KIST
    // const string ip_address = "192.168.0.9"; // cmd > ipconfig (WLAN IPV4 address) - home
    const string ip_address = "192.168.43.51"; // cmd > ipconfig (WLAN IPV4 address) - AndroidAP
    // const string ip_address = "192.168.0.2"; // cmd > ipconfig (LAN IPV4 address) - dorm
    //속도는 예를 들어 : 19775824 30 0 0 4 -> key velocity x y z (단위벡터)
    //회전은 예를 들어 : 38241913 1 0 0 0  -> key w x y z (쿼터니언)
    //시선은 예를 들어 : 22353818 1.5 -> key speed
    void Start()
    {
        Controller = GetComponent<CharacterController>();
        m_camera = Camera.main; // 1111
        client = new TcpClient();
        Application.runInBackground = true;
        client.Connect(ip_address, 23000);
        string hostName = Dns.GetHostName();
        string IPaddr = Dns.GetHostEntry(hostName).AddressList[2].ToString();
        NetworkStream stream = client.GetStream();
        stream.Write(Encoding.ASCII.GetBytes(IPaddr), 0, IPaddr.Length);
        Thread data_th = new Thread(receiveData);
        data_th.Start();
    }

    // Update is called once per frame
    void Update()
    {
        if (isStarted && strdata != "")
        // if (isStarted)
        {
            string[] splited = dataSplit(strdata);
            if (Equals(VELOCITY_KEY, splited[0])) fdatago(float.Parse(splited[1]), float.Parse(splited[2]), float.Parse(splited[3]), float.Parse(splited[4]));
            else if (Equals(ROTATION_KEY, splited[0])) rotateTo(new Quaternion(float.Parse(splited[2]), float.Parse(splited[3]), float.Parse(splited[4]), float.Parse(splited[1])));
            else if (Equals(GAZE_KEY, splited[0])) lookngo(float.Parse(splited[1]));
        }
    }
    string[] dataSplit(string RAW)
    {
        char sp = ' ';
        string[] tmp = RAW.Split(sp);
        return tmp;
    }

    void receiveData()
    {
        while (true)
        {
            try
            {
                if (isStarted)
                {
                    buffer = new byte[client.ReceiveBufferSize];
                    NetworkStream stream = client.GetStream();
                    stream.Read(buffer, 0, buffer.Length);
                    char[] formatted = new char[buffer.Length];
                    for (int i = 0; i < buffer.Length; i++)
                    {
                        formatted[i] = (char)buffer[i];
                    }
                    strdata = new string(formatted);
                    Debug.Log(strdata);

                }
                else isStarted = true;
            }
            catch { }
            Thread.Sleep(1);
        }
    }


    void fdatago(float velocity, float x , float y, float z) // FB : X , LR : Z
    {
        Debug.Log("Entered_fdatago");
        float speed = velocity;
        Controller.Move(new Vector3(x, y, z) * speed);
    }

    void rotateTo(Quaternion Q)
    {
        Debug.Log("Entered_rotateTo");
        Controller.transform.rotation = Q;
    }

    void lookngo(float speed)
    {
        Debug.Log("Entered_lookngo");
        Vector3 gaze = m_camera.transform.TransformDirection(0.0f, 0.0f, 1.0f);
        float gaze_length = Mathf.Sqrt(gaze.x*gaze.x + gaze.z*gaze.z);
        float gaze_x = 0.0f;
        float gaze_z = 0.0f;
        if (gaze_length > 1e-20)
        {
            gaze_x = gaze.x/gaze_length;
            gaze_z = gaze.z/gaze_length;
        }
        Controller.Move(new Vector3(gaze_x, 0.0f, gaze_z) * speed);
    }
}
