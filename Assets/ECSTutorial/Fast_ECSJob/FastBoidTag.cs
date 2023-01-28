using System.Collections;
using System.Collections.Generic;
using Unity.Entities;

namespace Tutorial.FastBiods
{
    public struct FastBoidTag : IComponentData {}
    public struct UnitBoidComponent : IComponentData
    {
        public int index;
    }
}