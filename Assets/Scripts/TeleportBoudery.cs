using UnityEngine;

public class TeleportBoudery : MonoBehaviour
{
    [SerializeField] private Boundery _boundery;
    private void FixedUpdate()
    {
        if (Mathf.Abs(transform.position.x) > _boundery.XLimit)
        {
            if (transform.position.x > 0)
            {
                transform.position = new Vector3(-_boundery.XLimit, transform.position.y, transform.position.z);
            }
            else
            {
                transform.position = new Vector3(_boundery.XLimit, transform.position.y, transform.position.z);
            }
        }
        if (Mathf.Abs(transform.position.y) > _boundery.YLimit)
        {
            if (transform.position.y > 0)
            {
                transform.position = new Vector3(transform.position.x, -_boundery.YLimit, transform.position.z);
            }
            else
            {
                transform.position = new Vector3(transform.position.x, _boundery.YLimit, transform.position.z);
            }
        }
    }
}