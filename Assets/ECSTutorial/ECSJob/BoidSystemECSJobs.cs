using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;

namespace Tutorial.Biods
{
    public partial class BoidSystemECSJobs : SystemBase
    {
        BoidControllerECSJobConponent controller;
        protected override void OnStartRunning()
        {
            Enabled = SystemAPI.HasSingleton<BoidControllerECSJobConponent>();
            if (Enabled == false)
            {
                Debug.Log("Disable BoidSystemECSJobs");
                return;
            }

            controller = SystemAPI.GetSingleton<BoidControllerECSJobConponent>();
            var ecb = World.GetExistingSystemManaged<BeginInitializationEntityCommandBufferSystem>().CreateCommandBuffer();


            //var boidArray = EntityManager.Instantiate(controller.prefab, controller.boidAmount, Allocator.TempJob);
            var boidArray = new NativeArray<Entity>(controller.boidAmount, Allocator.TempJob);
            ecb.Instantiate(controller.prefab, boidArray);

            for (int i = 0; i < boidArray.Length; i++)
            {
                Unity.Mathematics.Random rand = new Unity.Mathematics.Random((uint)i + 1);
                //---- LocalToWorld 안되는디?

                ecb.SetComponent<LocalTransform>(boidArray[i], new LocalTransform
                {
                    Position = RandomPosition(controller),//new float3(i) * 0.1f,
                    Rotation = RandomRotation(),//quaternion.identity,
                    Scale = 1
                });
                ecb.AddComponent<BoidECSJobs>(boidArray[i]);
            }

        }
        protected override void OnUpdate()
        {
            if (SystemAPI.HasSingleton<BoidControllerECSJobConponent>() == false)
                return;

            EntityQuery boidQuery = GetEntityQuery(ComponentType.ReadOnly<BoidECSJobs>(),
                ComponentType.ReadOnly<LocalTransform>());

            NativeArray<Entity> entityArray = boidQuery.ToEntityArray(Allocator.TempJob);
            NativeArray<LocalTransform> localToWorldArray = boidQuery.ToComponentDataArray<LocalTransform>(Allocator.TempJob);

            // These arrays get deallocated after job completion
            NativeArray<EntityWithLocalToWorld> boidArray = new NativeArray<EntityWithLocalToWorld>(entityArray.Length, Allocator.TempJob);
            //NativeArray<float3> boidPosition = new NativeArray<float3>(entityArray.Length, Allocator.TempJob);

            for (int i = 0; i < entityArray.Length; i++)
            {
                boidArray[i] = new EntityWithLocalToWorld
                {
                    entity = entityArray[i],
                    localTransform = localToWorldArray[i],
                    index = i
                };
            }

            entityArray.Dispose();
            localToWorldArray.Dispose();

            BoidJob boidJob = new BoidJob
            {
                otherBoids = boidArray,
                //boidPos = boidPosition,
                boidPerceptionRadius = controller.boidPerceptionRadius,
                separationWeight = controller.separationWeight,
                cohesionWeight = controller.cohesionWeight,
                alignmentWeight = controller.alignmentWeight,
                cageSize = controller.cageSize,
                avoidWallsTurnDist = controller.avoidWallsTurnDist,
                avoidWallsWeight = controller.avoidWallsWeight,
                boidSpeed = controller.boidSpeed,
                deltaTime = SystemAPI.Time.DeltaTime
            };

            var boidHandle = boidJob.ScheduleParallel(Dependency);
            boidHandle.Complete();

        }

        [BurstCompile]
        partial struct BoidJob : IJobEntity
        {
            [DeallocateOnJobCompletion] [ReadOnly] public NativeArray<EntityWithLocalToWorld> otherBoids;
            //[DeallocateOnJobCompletion] 이 자동으로 Dispose 하는거 같은데?

            [ReadOnly] public float boidPerceptionRadius;
            [ReadOnly] public float separationWeight;
            [ReadOnly] public float cohesionWeight;
            [ReadOnly] public float alignmentWeight;
            [ReadOnly] public float cageSize;
            [ReadOnly] public float avoidWallsTurnDist;
            [ReadOnly] public float avoidWallsWeight;
            [ReadOnly] public float boidSpeed;
            [ReadOnly] public float deltaTime;

            public void Execute(Entity boid, ref LocalTransform trans)
            {
                float3 boidPosition = trans.Position;

                float3 seperationSum = float3.zero;
                float3 positionSum = float3.zero;
                float3 headingSum = float3.zero;

                int boidsNearby = 0;
                int index = -1;

                for (int otherBoidIndex = 0; otherBoidIndex < otherBoids.Length; otherBoidIndex++)
                {
                    if (boid != otherBoids[otherBoidIndex].entity)
                    {

                        float3 otherPosition = otherBoids[otherBoidIndex].localTransform.Position;
                        float distToOtherBoid = math.length(boidPosition - otherPosition);

                        if (distToOtherBoid < boidPerceptionRadius)
                        {

                            seperationSum += -(otherPosition - boidPosition) * (1f / math.max(distToOtherBoid, .0001f));
                            positionSum += otherPosition;
                            headingSum += otherBoids[otherBoidIndex].localTransform.Forward();

                            boidsNearby++;
                        }
                    }
                    else
                    {
                        index = otherBoidIndex;
                    }
                }//자기 자신이 아닌 대상에게 , 일정 범위안이면

                float3 force = float3.zero;

                if (boidsNearby > 0)
                {
                    force += (seperationSum / boidsNearby) * separationWeight;
                    force += ((positionSum / boidsNearby) - boidPosition) * cohesionWeight;
                    force += (headingSum / boidsNearby) * alignmentWeight;
                }
                if (math.min(math.min(
                    (cageSize / 2f) - math.abs(boidPosition.x),
                    (cageSize / 2f) - math.abs(boidPosition.y)),
                    (cageSize / 2f) - math.abs(boidPosition.z))
                        < avoidWallsTurnDist)
                {
                    force += -math.normalize(boidPosition) * avoidWallsWeight;
                }//범위 안으로 유지

                float3 velocity = trans.Forward() * boidSpeed;
                velocity += force * deltaTime;
                velocity = math.normalize(velocity) * boidSpeed;

                trans = new LocalTransform
                {
                    Position = trans.Position + velocity * deltaTime,
                    Rotation = quaternion.LookRotationSafe(velocity, trans.Up()),
                    Scale = 1
                };

                //boidPos[index] = trans.Position + velocity * deltaTime;
            }
        }

        [BurstCompile]
        private partial struct BoidMoveJob : IJobEntity
        {

            [DeallocateOnJobCompletion] [ReadOnly] public NativeArray<float4x4> newBoidTransforms;

            public void Execute(Entity boid, int boidIndex, [WriteOnly] ref LocalToWorld localToWorld)
            {
                localToWorld.Value = newBoidTransforms[boidIndex];
            }
        }



        private float3 RandomPosition(BoidControllerECSJobConponent ControllerData)
        {
            return new float3(
                UnityEngine.Random.Range(-ControllerData.cageSize / 2f, ControllerData.cageSize / 2f),
                UnityEngine.Random.Range(-ControllerData.cageSize / 2f, ControllerData.cageSize / 2f),
                UnityEngine.Random.Range(-ControllerData.cageSize / 2f, ControllerData.cageSize / 2f)
            );
        }
        private quaternion RandomRotation()
        {
            return quaternion.Euler(
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f)
            );
        }
    }

}
