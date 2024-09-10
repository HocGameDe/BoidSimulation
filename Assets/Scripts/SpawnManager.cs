using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnManager : MonoBehaviour
{
    [SerializeField] private ListBoidVariable boids;
    [SerializeField] private GameObject boidPrefab;
    [SerializeField] private int boidCount = 10;

    private void Awake()
    {
        if (boids.boidTransform.Count > 0) boids.boidTransform.Clear();

     
        for (int i = 0; i < boidCount; i++)
        {
            float direction = Random.Range(0f, 360f);

            Vector3 position = new Vector2(Random.Range(-20f, 20f), Random.Range(-20f, 20f));
            GameObject boid = Instantiate(boidPrefab, position,
                Quaternion.Euler(Vector3.forward * direction) * boidPrefab.transform.localRotation);
            //boid.transform.SetParent(transform);
            boids.boidTransform.Add(boid.transform);
        }
    }

}
