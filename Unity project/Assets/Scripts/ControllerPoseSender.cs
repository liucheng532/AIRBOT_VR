using UnityEngine;
using System.Collections.Generic;
using UnityEngine.XR;
using System;
using System.Net.Sockets;
using System.Text;

public class ControllerData : MonoBehaviour
{
    InputDevice deviceLeft;
    InputDevice deviceRight;

    public int port = 8000;
    public string ipAddress = "10.119.121.140";

    private float sendInterval = 0.05f; // 20Hz
    private float timeSinceLastSend = 0f;

    private bool sending = false;                 // ÈÎÒâ Grip¡°ÉÏÉýÑØ¡±¿ªÊŒ£»Í£Ö¹=×óY
    private bool leftPaused = false, rightPaused = false; // ÔÝÍ££º×óX / ÓÒA£š°Ž×¡¶³œá£©

    private bool prevLGrip = false, prevRGrip = false;

    private Vector3 lastLPos = Vector3.zero, lastRPos = Vector3.zero;
    private Vector3 lastLEul = Vector3.zero, lastREul = Vector3.zero;
    private bool lastLValid = false, lastRValid = false;

    void Start()
    {
        deviceLeft = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        deviceRight = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        if (!deviceLeft.isValid) Debug.LogWarning("Could not find left hand device.");
        if (!deviceRight.isValid) Debug.LogWarning("Could not find right hand device.");

        InputDevices.deviceConnected += OnDeviceChanged;
        InputDevices.deviceDisconnected += OnDeviceChanged;
    }

    void OnDestroy()
    {
        InputDevices.deviceConnected -= OnDeviceChanged;
        InputDevices.deviceDisconnected -= OnDeviceChanged;
    }

    void OnDeviceChanged(InputDevice _)
    {
        deviceLeft = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        deviceRight = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
    }

    void Update()
    {
        // ¡ª¡ª Grip ÉÏÉýÑØŽ¥·¢¿ªÊŒ ¡ª¡ª 
        bool lGripNow = TryGrip(deviceLeft, out float lGripVal) && lGripVal > 0.1f;
        bool rGripNow = TryGrip(deviceRight, out float rGripVal) && rGripVal > 0.1f;
        bool rising = (!prevLGrip && lGripNow) || (!prevRGrip && rGripNow);
        if (rising) sending = true;
        prevLGrip = lGripNow; prevRGrip = rGripNow;

        // ¡ª¡ª Í£Ö¹£º×óÊÖ Y ¡ª¡ª 
        if (deviceLeft.isValid &&
            deviceLeft.TryGetFeatureValue(CommonUsages.secondaryButton, out bool leftY) && leftY)
        {
            SendDataToPC("EXIT=T\n");
            sending = false;
        }

        // ¡ª¡ª ÔÝÍ££º×ó X / ÓÒ A ¡ª¡ª 
        leftPaused = deviceLeft.isValid &&
                      deviceLeft.TryGetFeatureValue(CommonUsages.primaryButton, out bool leftX) && leftX;
        rightPaused = deviceRight.isValid &&
                      deviceRight.TryGetFeatureValue(CommonUsages.primaryButton, out bool rightA) && rightA;

        // °ŽÏÂÊ±¶îÍâ·¢ËÍÊÂŒþÐÐ
        if (leftPaused) SendDataToPC("PauseL=T\n");
        if (rightPaused) SendDataToPC("PauseR=T\n");

        // ¡ª¡ª ¶šÊ±·¢ËÍ ¡ª¡ª 
        timeSinceLastSend += Time.deltaTime;
        if (timeSinceLastSend >= sendInterval)
        {
            if (sending) SendControllerData();
            timeSinceLastSend = 0f;
        }
    }

    void SendControllerData()
    {
        string leftData = "";
        string rightData = "";
        string LGripStatus = "LGrip=F";
        string RGripStatus = "RGrip=F";
        string LTrigger = "LTrig=F";
        string RTrigger = "RTrig=F";

        Vector3 lPos = Vector3.zero, rPos = Vector3.zero;
        Quaternion lRot = Quaternion.identity, rRot = Quaternion.identity;

        bool lPosOk = deviceLeft.isValid && deviceLeft.TryGetFeatureValue(CommonUsages.devicePosition, out lPos);
        bool lRotOk = deviceLeft.isValid && deviceLeft.TryGetFeatureValue(CommonUsages.deviceRotation, out lRot);
        bool rPosOk = deviceRight.isValid && deviceRight.TryGetFeatureValue(CommonUsages.devicePosition, out rPos);
        bool rRotOk = deviceRight.isValid && deviceRight.TryGetFeatureValue(CommonUsages.deviceRotation, out rRot);

        if (TryGrip(deviceLeft, out float lGripVal2)) LGripStatus = lGripVal2 > 0.1f ? "LGrip=T" : "LGrip=F";
        if (TryGrip(deviceRight, out float rGripVal2)) RGripStatus = rGripVal2 > 0.1f ? "RGrip=T" : "RGrip=F";
        if (TryTrigger(deviceLeft, out float lTrig2)) LTrigger = lTrig2 > 0.1f ? "LTrig=T" : "LTrig=F";
        if (TryTrigger(deviceRight, out float rTrig2)) RTrigger = rTrig2 > 0.1f ? "RTrig=T" : "RTrig=F";

        Vector3 sendLPos = lPos; Vector3 sendLEul = lRot.eulerAngles; bool useL = lPosOk && lRotOk;
        Vector3 sendRPos = rPos; Vector3 sendREul = rRot.eulerAngles; bool useR = rPosOk && rRotOk;

        if (leftPaused && lastLValid) { sendLPos = lastLPos; sendLEul = lastLEul; useL = true; }
        else if (lPosOk && lRotOk) { lastLPos = lPos; lastLEul = lRot.eulerAngles; lastLValid = true; }

        if (rightPaused && lastRValid) { sendRPos = lastRPos; sendREul = lastREul; useR = true; }
        else if (rPosOk && rRotOk) { lastRPos = rPos; lastREul = rRot.eulerAngles; lastRValid = true; }

        if (useL) leftData = $"LPos: {sendLPos.ToString("F6")} LRot: {sendLEul.ToString("F6")}";
        if (useR) rightData = $"RPos: {sendRPos.ToString("F6")} RRot: {sendREul.ToString("F6")}";

        string combinedData = $"{LGripStatus};{RGripStatus};{LTrigger};{RTrigger};{leftData};{rightData};\n";
        SendDataToPC(combinedData);
    }

    private void SendDataToPC(string data)
    {
        try
        {
            using (TcpClient client = new TcpClient())
            {
                client.NoDelay = true;
                client.Connect(ipAddress, port);
                NetworkStream stream = client.GetStream();
                byte[] dataBytes = Encoding.ASCII.GetBytes(data);
                stream.Write(dataBytes, 0, dataBytes.Length);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error sending data to PC: " + e.Message);
        }
    }

    private bool TryGrip(InputDevice dev, out float val)
    {
        val = 0f;
        if (!dev.isValid) return false;
        return dev.TryGetFeatureValue(CommonUsages.grip, out val);
    }

    private bool TryTrigger(InputDevice dev, out float val)
    {
        val = 0f;
        if (!dev.isValid) return false;
        return dev.TryGetFeatureValue(CommonUsages.trigger, out val);
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 1000, 25),
            $"sending={sending}  leftPaused={leftPaused}  rightPaused={rightPaused}  (Start: any Grip | Pause: Left X / Right A | Stop: Left Y)");
    }
}