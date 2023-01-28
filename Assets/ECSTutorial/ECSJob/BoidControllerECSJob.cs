using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace Tutorial.Biods
{
    public class BoidControllerECSJob : MonoBehaviour
    {
        public static BoidControllerECSJob Instance;
        public int boidAmount = 1000;
        public Mesh sharedMesh;
        public Material sharedMaterial;

        public GameObject Prefab;
        public float OffsetScale = 1f;
        public float boidSpeed = 10f;
        public float boidPerceptionRadius = 5;
        public float cageSize = 10;

        public float separationWeight = 25;
        public float cohesionWeight = 5;
        public float alignmentWeight = 10;

        public float avoidWallsWeight = 10;
        public float avoidWallsTurnDist = 5;

        //https://github.com/BadGraphixD/How-many-Boids-can-Unity-handle/blob/master/Assets/Scenes/3)%20ECS%20%2B%20Jobs/BoidControllerECSJobs.cs

        void Start()
        {
            if (Instance == null)
                Instance = this;
        }

        private void OnDrawGizmos()
        {

            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(
                Vector3.zero,
                new Vector3(
                    cageSize,
                    cageSize,
                    cageSize
                )
            );
        }
    }

    public struct BoidControllerECSJobConponent : IComponentData
    {
        public Entity prefab;
        public float offsetScale;
        public int boidAmount;
        public float boidSpeed;
        public float boidPerceptionRadius;
        public float cageSize;

        public float separationWeight;
        public float cohesionWeight;
        public float alignmentWeight;

        public float avoidWallsWeight;
        public float avoidWallsTurnDist;
    }

    public class BoidControllerECSJobBaker : Baker<BoidControllerECSJob>
    {
        public override void Bake(BoidControllerECSJob authoring)
        {
            AddComponent(new BoidControllerECSJobConponent
            {
                prefab = GetEntity(authoring.Prefab),
                offsetScale = authoring.OffsetScale,
                boidAmount = authoring.boidAmount,
                boidSpeed = authoring.boidSpeed,
                boidPerceptionRadius = authoring.boidPerceptionRadius,
                cageSize = authoring.cageSize,

                separationWeight = authoring.separationWeight,
                cohesionWeight = authoring.cohesionWeight,
                alignmentWeight = authoring.alignmentWeight,

                avoidWallsWeight = authoring.avoidWallsWeight,
                avoidWallsTurnDist = authoring.avoidWallsTurnDist
            });
        }
    }
}
