#include "Octree.h"
#include "niven.Core.Math.ArrayFunctions.h"
#include "niven.Core.Math.VectorFunctions.h"

Octree::Octree()
        : sampleCount_(0)
{
	memset(child_, 0, sizeof(child_));
}

bool Octree::Build(std::vector<GuidanceFieldSample> samples,
							  const std::vector<niven::Vector3f>& offsetTable,
                              const unsigned int count,
                              const OctreeBoundingBox &bounds)
{
		// Set Octree information
		sampleCount_ = count;
		boundingSphere_ = bounds;
		// Calculate the bounding box for efficient traversal later on
		// calculateBoundingSphere(samples, boundingSphere_);

        // You know you're a leaf when the number of points is <= 1
        if (count == 1)
        {
			// Just store the sample in the node, making it a leaf
			sample_ = samples.front();
			// Set the bounding sphere position to this samples position
			boundingSphere_.radius = 0.0f;
			boundingSphere_.center = sample_.position;
			return true;
        }

        // Classify each sample to a child node

		// Center of this node
		const niven::Vector3f &c = bounds.center;
		std::vector<int> childPointCounts(8);

        for (unsigned int i = 0; i < count; i++)
        {
                // Current sample
                GuidanceFieldSample &sample = samples[i];

                // Here, we need to know which child each sample belongs to. To
                // do this, we build an index into the _child[] array using the
                // relative position of the sample to the center of the current
                // node

                sample.childCode = 0;
				if (sample.position.X() > c.X()) sample.childCode |= 1;
                if (sample.position.Y() > c.Y()) sample.childCode |= 2;
                if (sample.position.Z() > c.Z()) sample.childCode |= 4;

                // We'll need to keep track of how many samples get stuck in each
                // child so we'll just keep track of it here, since we have the
                // information handy.

                childPointCounts[sample.childCode]++;
        }

        // Recursively call build() for each of the 8 children

        for (unsigned int i = 0; i < 8; i++)
        {
                // Don't bother going any further if there aren't any points for
                // this child

                if (childPointCounts[i] == 0) continue;

                // Allocate the child
                child_[i] = new Octree();

                // Allocate a list of samples that were coded JUST for this child
                // only

                std::vector<GuidanceFieldSample> childSamples;

                // Go through the input list of samples and copy over the samples
                // that were coded for this child

                for (unsigned int j = 0; j < count; j++)
                {
					if (samples[j].childCode == i)
                        {
							childSamples.push_back(samples[j]);
                        }
                }

                // Calculate our offset from the center of the parent's node to
                // the center of the child's node

				niven::Vector3f offset = offsetTable[i] * bounds.radius;

                // Create a new Bounds, with the center offset and half the
                // radius

                OctreeBoundingBox newBounds;
                newBounds.radius = bounds.radius * 0.5;
                newBounds.center = bounds.center + offset;

                // Recurse

                child_[i]->Build(childSamples, offsetTable, childPointCounts[i],
                                newBounds);
        }

        return true;
}

void Octree::Traverse(TraversalFunction& proc, const Vector3f& pos, float distance, std::vector<GuidanceFieldSample>& data) const
{
        // Call the callback for this node (if the callback returns false, then
        // stop traversing.

        if (!proc(*this, pos, distance, data)) return;

        // If I'm a node, recursively traverse my children

        if (sampleCount_ > 1)
        {
                for (unsigned int i = 0; i < 8; i++)
                {
                        // We store incomplete trees (i.e. we're not guaranteed
                        // that a node has all 8 children)

                        if (!child_[i]) continue;

                        child_[i]->Traverse(proc, pos, distance, data);
                }
        }
}

void Octree::calculateBoundingSphere(std::vector<GuidanceFieldSample>& samples, OctreeBoundingBox& bounds) {

	Vector3f minPos = Vector3f::Constant(std::numeric_limits<float>::max());
	Vector3f maxPos = Vector3f::Constant(std::numeric_limits<float>::min());

	for (unsigned int i = 0; i < samples.size(); i++)
		{
			const GuidanceFieldSample &g = samples[i];
			if (g.position.X() < minPos.X()) minPos.X() = g.position.X();
			if (g.position.Y() < minPos.Y()) minPos.Y() = g.position.Y();
			if (g.position.Z() < minPos.Z()) minPos.Z() = g.position.Z();
			if (g.position.X() > maxPos.X()) maxPos.X() = g.position.X();
			if (g.position.Y() > maxPos.Y()) maxPos.Y() = g.position.Y();
			if (g.position.Z() > maxPos.Z()) maxPos.Z() = g.position.Z();
        }

	Vector3f radius = maxPos - minPos;

	bounds.center = minPos + radius * 0.5;

	// Find the maximum radius, because we want a cubic bounding box
	bounds.radius = radius.X();
    if (bounds.radius < radius.Y()) bounds.radius = radius.Y();
    if (bounds.radius < radius.Z()) bounds.radius = radius.Z();
}

bool Octree::IsLeaf() const {
	return (sampleCount_ == 1);
}

GuidanceFieldSample Octree::GetLeafSample()const {
	return sample_;
}

bool Octree::PositionIsWithinDistance(const Vector3f& pos, float distance) const {
	return NormInfinity(boundingSphere_.center - pos) <= (boundingSphere_.radius + distance);
}
