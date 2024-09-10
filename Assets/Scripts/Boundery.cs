using UnityEngine;

[CreateAssetMenu(menuName = "ScriptableObject/Boundery")]
public class Boundery : ScriptableObject
{
    private float _xLimit;
    private float _yLimit;
    public float XLimit
    {
        get
        {
            CalculateLimit();
            return _xLimit;
        }
    }
    public float YLimit
    {
        get
        {
            CalculateLimit();
            return _yLimit;
        }
    }
    private void CalculateLimit()
    {
        _yLimit = Camera.main.orthographicSize + 1f;
        _xLimit = _yLimit * Screen.width / Screen.height + 1f;
    }
}