using Unity.Entities;
using Unity.Transforms;

namespace Tutorial.Biods
{
    public struct BoidECSJobs : IComponentData { }

    public struct EntityWithLocalToWorld : IComponentData
    {
        public Entity entity;
        public LocalTransform localTransform;//public LocalToWorld localToWorld;

        public int index;
    }
}
