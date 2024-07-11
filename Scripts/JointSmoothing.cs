using Microsoft.Azure.Kinect.BodyTracking;
using System.Collections.Generic;
using UnityEngine;

public class JointSmoothing
{
    private Dictionary<JointId, Queue<Vector3>> jointPositionQueues;
    private Dictionary<JointId, Queue<Quaternion>> jointRotationQueues;
    private int smoothingWindow;

    public JointSmoothing(int windowSize)
    {
        smoothingWindow = windowSize;
        jointPositionQueues = new Dictionary<JointId, Queue<Vector3>>();
        jointRotationQueues = new Dictionary<JointId, Queue<Quaternion>>();
        foreach (JointId joint in System.Enum.GetValues(typeof(JointId)))
        {
            jointPositionQueues[joint] = new Queue<Vector3>();
            jointRotationQueues[joint] = new Queue<Quaternion>();
        }
    }

    public Vector3 GetSmoothedPosition(JointId jointId, Vector3 newPosition)
    {
        Queue<Vector3> positions = jointPositionQueues[jointId];
        if (positions.Count >= smoothingWindow)
        {
            positions.Dequeue();
        }
        positions.Enqueue(newPosition);

        Vector3 smoothedPosition = Vector3.zero;
        foreach (Vector3 pos in positions)
        {
            smoothedPosition += pos;
        }
        return smoothedPosition / positions.Count;
    }

    public Quaternion GetSmoothedRotation(JointId jointId, Quaternion newRotation)
    {
        Queue<Quaternion> rotations = jointRotationQueues[jointId];
        if (rotations.Count >= smoothingWindow)
        {
            rotations.Dequeue();
        }
        rotations.Enqueue(newRotation);

        Quaternion smoothedRotation = Quaternion.identity;
        foreach (Quaternion rot in rotations)
        {
            smoothedRotation = Quaternion.Slerp(smoothedRotation, rot, 1.0f / rotations.Count);
        }
        return smoothedRotation;
    }
}
