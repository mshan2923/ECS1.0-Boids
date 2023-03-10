
/** 
* Parts of this code were originally made by Bogdan Codreanu
* Original code: https://github.com/BogdanCodreanu/ECS-Boids-Murmuration_Unity_2019.1
*/ 

using Unity.Jobs;
using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using Samples.Boids;
using UnityEngine;

namespace Tutorial.FastBiods
{
    [BurstCompile]
    public partial class HashBoidSystem : SystemBase
    {
        private EntityQuery boidGroup;
        private HashBiodControllerConponent controller;

        EntityCommandBuffer ecb;
        
#region Job
        // Copies all boid positions and headings into buffer
        [BurstCompile]
        private partial struct CopyPositionsAndHeadingsInBuffer : IJobEntity
        {
            public NativeArray<float3> boidPositions;
            public NativeArray<float3> boidHeadings;

            public void Execute(Entity boid, [EntityIndexInQuery] int boidIndex, in LocalTransform trans , in UnitBoidComponent unit) 
            {
                boidPositions[boidIndex] = trans.Position;
                boidHeadings[boidIndex] = trans.Forward();
            }
        }

        // Asigns each boid to a cell. Each boid index is stored in the hashMap. Each hash corresponds to a cell.
        // The cell grid has a random offset and rotation each frame to remove artefacts.
        [BurstCompile]
        partial struct HashPositionToHashMap : IJobEntity
        {
            public NativeMultiHashMap<int, int>.ParallelWriter hashMap;
            [ReadOnly] public quaternion cellRotationVary;
            [ReadOnly] public float3 positionOffsetVary;
            [ReadOnly] public float cellRadius;

            public void Execute(Entity boid, [EntityIndexInQuery] int boidIndex, in UnitBoidComponent unit, in LocalTransform trans) 
            {
                var hash = (int)math.hash(new int3(math.floor(math.mul(cellRotationVary, trans.Position + positionOffsetVary) / cellRadius)));
                hashMap.Add(hash, boidIndex);
            }
        }

        // Sums up positions and headings of all boids of each cell. These sums are stored in the
        // same array as before (cellPositions and cellHeadings), so that there is no need for
        // a new array. The index of each cell is set to the index of the first boid.
        // With the array indicesOfCells each boid can find the index of its cell.
        // This way every boid knows the sum of all the positions (and headings) of all the other
        // boids in the same cell -> no nested loop required -> massive performance boost
        [BurstCompile]
        partial struct MergeCellJob : Samples.Boids.IJobNativeMultiHashMapMergedSharedKeyIndices
        {
            public NativeArray<int> indicesOfCells;
            public NativeArray<float3> cellPositions;
            public NativeArray<float3> cellHeadings;
            public NativeArray<int> cellCount;
            public void ExecuteFirst(int firstBoidIndexEncountered)
            {
                indicesOfCells[firstBoidIndexEncountered] = firstBoidIndexEncountered;
                cellCount[firstBoidIndexEncountered] = 1;
                float3 positionInThisCell = cellPositions[firstBoidIndexEncountered] / cellCount[firstBoidIndexEncountered];
            }

            public void ExecuteNext(int firstBoidIndexAsCellKey, int boidIndexEncountered)
            {
                cellCount[firstBoidIndexAsCellKey] += 1;
                cellHeadings[firstBoidIndexAsCellKey] += cellHeadings[boidIndexEncountered];
                cellPositions[firstBoidIndexAsCellKey] += cellPositions[boidIndexEncountered];
                indicesOfCells[boidIndexEncountered] = firstBoidIndexAsCellKey;
            }
        }

        // Calculates the forces for each boid (no nested loop). All forces are weighted, added up
        // and directly applied to orientation and position.
        [BurstCompile]
        partial struct MoveBoids : IJobEntity
        {
            [ReadOnly] public float deltaTime;
            [ReadOnly] public float boidSpeed;

            [ReadOnly] public float separationWeight;
            [ReadOnly] public float alignmentWeight;
            [ReadOnly] public float cohesionWeight;

            [ReadOnly] public float cageSize;
            [ReadOnly] public float cageAvoidDist;
            [ReadOnly] public float cageAvoidWeight;

            [ReadOnly] public float cellSize;
            [DeallocateOnJobCompletion, ReadOnly] public NativeArray<int> cellIndices;
            [DeallocateOnJobCompletion, ReadOnly] public NativeArray<float3> positionSumsOfCells;
            [DeallocateOnJobCompletion, ReadOnly] public NativeArray<float3> headingSumsOfCells;
            [DeallocateOnJobCompletion, ReadOnly] public NativeArray<int> cellBoidCount;
            public void Execute(Entity boid, [EntityIndexInQuery] int boidIndex, in UnitBoidComponent unit, ref LocalTransform trans)
            {
                //if (boidIndex >= cellIndices.Length)
                //    return;//! ????????? cellIndices.Length ??? 0 
                    //UnityEngine.Debug.Log("cellIndices Out Of Length");

                float3 boidPosition = trans.Position;
                int cellIndex = cellIndices[boidIndex];//??? Cell?????? ?????? ???????????? ????????? ??????????????? ???????????? ??????

                if (cellIndex >= cellBoidCount.Length || cellIndex < 0)
                {
                    return;//!SECTION ===============  ???????????? ??????...
                }

                int nearbyBoidCount = cellBoidCount[cellIndex] - 1;
                float3 positionSum = positionSumsOfCells[cellIndex] - trans.Position;
                float3 headingSum = headingSumsOfCells[cellIndex] - trans.Forward();

                float3 force = float3.zero;

                if (nearbyBoidCount > 0)
                {
                    float3 averagePosition = positionSum / nearbyBoidCount;

                    float distToAveragePositionSq = math.lengthsq(averagePosition - boidPosition);
                    float maxDisToAveragePositionSq = cellSize * cellSize;

                    float distanceNormalized = distToAveragePositionSq / maxDisToAveragePositionSq;
                    float needToLeave = math.max(1 - distanceNormalized, 0);

                    float3 toAveragePosition = math.normalizesafe(averagePosition - boidPosition);
                    float3 averageHeading = headingSum / nearbyBoidCount;

                    force += -toAveragePosition * separationWeight * needToLeave;
                    force +=  toAveragePosition * cohesionWeight;
                    force +=  averageHeading    * alignmentWeight;
                }

                if (math.min(math.min(
                (cageSize / 2f) - math.abs(boidPosition.x),
                (cageSize / 2f) - math.abs(boidPosition.y)),
                (cageSize / 2f) - math.abs(boidPosition.z))
                    < cageAvoidDist) 
                {
                    force += -math.normalize(boidPosition) * cageAvoidWeight;
                }

                float3 velocity = trans.Forward() * boidSpeed;
                velocity += force * deltaTime;
                velocity = math.normalize(velocity) * boidSpeed;

                trans = new LocalTransform
                {
                    Position = trans.Position + velocity * deltaTime,
                    Rotation = quaternion.LookRotationSafe(velocity, trans.Up()),
                    Scale = 1
                };
            }//! cellIndices OutOfLength 
        }
#endregion

        protected override void OnStartRunning()
        {
            if (SystemAPI.HasSingleton<HashBiodControllerConponent>() == false)
            {
                Enabled = false;
                return;
            }
            

            controller = SystemAPI.GetSingleton<HashBiodControllerConponent>();
            ecb = World.GetExistingSystemManaged<BeginInitializationEntityCommandBufferSystem>().CreateCommandBuffer();

            var boidArray = new NativeArray<Entity>(controller.boidAmount, Allocator.TempJob);
            ecb.Instantiate(controller.prefab, boidArray);

            for (int i = 0; i < boidArray.Length; i++)
            {
                Unity.Mathematics.Random rand = new Unity.Mathematics.Random((uint)i + 1);
                //---- LocalToWorld ?????????????

                ecb.SetComponent<LocalTransform>(boidArray[i], new LocalTransform
                {
                    Position = RandomPosition(controller),//new float3(i) * 0.1f,
                    Rotation = RandomRotation(),//quaternion.identity,
                    Scale = 1
                });
                ecb.AddComponent<FastBoidTag>(boidArray[i]);//BoidECSJobsFast >> FastBoidTag
                ecb.AddComponent<UnitBoidComponent>(boidArray[i], new UnitBoidComponent{index = i});
            }

            boidGroup = GetEntityQuery(new EntityQueryDesc 
            {
                All = new[] { ComponentType.ReadOnly<FastBoidTag>(), ComponentType.ReadWrite<LocalTransform>() },
                Options = EntityQueryOptions.FilterWriteGroup
            });

            //var query = GetEntityQuery(typeof(FastBoidTag));
            //Debug.Log("boid Amount : " + boidGroup.CalculateEntityCount() + " | " + query.CalculateEntityCount() + "\n" + boidGroup.Equals(query));
            //  ==> ?????? : boid Amount : 0 | 0 \n false
        }
        protected override void OnUpdate()
        {
            if (Enabled == false)
                return;

                // controller.boidPerceptionRadius ??? Cell?????? ??????

                // CopyPositionsAndHeadingsInBuffer ?????? ????????? ????????? ?????? 
                // hashPositionJobHandle ??????  HashMap??? ?????? ,  (?????? * (????????? + ????????????))??? ????????? ?????? int??? ?????? HashMap??? ??????
                //          ?????????????????? ?????? ???????????? ?????? ????????? ?????? ??????
                // MergeCellJob ?????? cell?????? ?????? ????????? ?????? , ????????? ???
                //

            int boidCount = boidGroup.CalculateEntityCount();
            if (boidCount == 0)
            {                
                boidGroup = GetEntityQuery(new EntityQueryDesc 
                {
                    All = new[] { ComponentType.ReadOnly<FastBoidTag>(), ComponentType.ReadWrite<LocalTransform>() },
                    Options = EntityQueryOptions.FilterWriteGroup
                });

                //boidGroup = GetEntityQuery(typeof(FastBoidTag));//?????? ?????? ????????? ???????????? ??????????????? ???
                Debug.Log("boidCount is 0");
                return;
            }

            var cellIndices = new NativeArray<int>(boidCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var cellBoidCount = new NativeArray<int>(boidCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var boidPositions = new NativeArray<float3>(boidCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var boidHeadings = new NativeArray<float3>(boidCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var hashMap = new NativeMultiHashMap<int, int>(boidCount, Allocator.TempJob);

            var positionsAndHeadingsCopyJob = new CopyPositionsAndHeadingsInBuffer {
                boidPositions = boidPositions,
                boidHeadings = boidHeadings
            };
            JobHandle positionAndHeadingsCopyJobHandle = positionsAndHeadingsCopyJob.ScheduleParallel(boidGroup, Dependency);


            quaternion randomHashRotation = quaternion.Euler
            (
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f)
            );
            float offsetRange = controller.boidPerceptionRadius / 2f;
            float3 randomHashOffset = new float3
            (
                UnityEngine.Random.Range(-offsetRange, offsetRange),
                UnityEngine.Random.Range(-offsetRange, offsetRange),
                UnityEngine.Random.Range(-offsetRange, offsetRange)
            );

            var hashPositionsJob = new HashPositionToHashMap {
                hashMap = hashMap.AsParallelWriter(),
                cellRotationVary = randomHashRotation,
                positionOffsetVary = randomHashOffset,
                cellRadius = controller.boidPerceptionRadius,
            };
            JobHandle hashPositionJobHandle = hashPositionsJob.ScheduleParallel(boidGroup, Dependency);


            // Proceed when these two jobs have been completed
            JobHandle copyAndHashJobHandle = JobHandle.CombineDependencies(
                positionAndHeadingsCopyJobHandle,
                hashPositionJobHandle
            );
            
            if (cellIndices.Length == 0)
            {
                Debug.Log("before mergeCellsJob , cellIndices Size is 0 ");
            }
            
            var mergeCellsJob = new MergeCellJob {
                indicesOfCells = cellIndices,
                cellPositions = boidPositions,
                cellHeadings = boidHeadings,
                cellCount = cellBoidCount,
            };
            JobHandle mergeCellJobHandle = mergeCellsJob.Schedule(hashMap, 64, copyAndHashJobHandle);

            
            if (cellIndices.Length == 0)
                Debug.Log("before moveJob , cellIndices Size is 0 ");

            var moveJob = new MoveBoids {
                deltaTime = SystemAPI.Time.DeltaTime,
                boidSpeed = controller.boidSpeed,

                separationWeight = controller.separationWeight,
                alignmentWeight = controller.alignmentWeight,
                cohesionWeight = controller.cohesionWeight,

                cageSize = controller.cageSize,
                cageAvoidDist = controller.avoidWallsTurnDist,
                cageAvoidWeight = controller.avoidWallsWeight,

                cellSize = controller.boidPerceptionRadius,
                cellIndices = cellIndices,
                positionSumsOfCells = boidPositions,
                headingSumsOfCells = boidHeadings,
                cellBoidCount = cellBoidCount,
            };
            JobHandle moveJobHandle = moveJob.ScheduleParallel(boidGroup, mergeCellJobHandle);
            moveJobHandle.Complete();

            {
                //cellIndices.Dispose();
                //cellBoidCount.Dispose();
                //boidPositions.Dispose();
                //boidHeadings.Dispose();// MoveJob?????? [DeallocateOnJobCompletion] ????????? ???????????? ??????
                hashMap.Dispose();
            }

            Dependency = moveJobHandle;
            boidGroup.AddDependency(Dependency);            
        }

        private float3 RandomPosition(HashBiodControllerConponent ControllerData)
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