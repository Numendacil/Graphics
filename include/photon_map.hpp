#ifndef PHOTON_MAP_H
#define PHOTON_MAP_H

#include <vecmath.h>
#include <cmath>
#include <algorithm>
#include <queue>
#include <numeric>
#include "utils.hpp"

struct Photon
{
	Vector3f pos;
	Vector3f dir;
	Vector3f power;
};

struct Node
{
	int idx;
	int axis;
	Node* left;
	Node* right;
};

class KdTree
{
private:
	Node* root;
	int num;
	Photon* Photons;

	Node* KdNodeBuild(int* IdxArray, int n, int level)
	{
		if (n <= 0)
			return nullptr;
		int axis = level % 3;
		std::sort(IdxArray, IdxArray + n, [&](const int a, const int b) {return this->Photons[a].pos[axis] < this->Photons[b].pos[axis];});
		int mid = (n - 1) / 2;
		Node* pthis = new Node;
		pthis->axis = axis;
		pthis->idx = IdxArray[mid];
		pthis->left = KdNodeBuild(IdxArray, mid, level + 1);
		pthis->right = KdNodeBuild(IdxArray + mid + 1, n - mid - 1, level + 1);
		return pthis;
	}

	void SearchKNearestNode(Node* node, const Vector3f& target, int k, std::priority_queue<std::pair<float, int>>& queue)
	{
		if (node == nullptr)
			return;
		float dist = (target - this->Photons[node->idx].pos).squaredLength();
		queue.emplace(dist, node->idx);
		if (queue.size() > k)
			queue.pop();
		float diff = target[node->axis] - this->Photons[node->idx].pos[node->axis];
		if (diff < 0)
			SearchKNearestNode(node->left, target, k, queue);
		else
			SearchKNearestNode(node->right, target, k, queue);
		if (queue.top().first > diff * diff || queue.size() < k)
		{
			if (diff < 0)
				SearchKNearestNode(node->right, target, k, queue);
			else
				SearchKNearestNode(node->left, target, k, queue);
		}
		
	}

	void SearchNodeInRange(Node* node, const Vector3f& target, float radius2, std::vector<int>& list)
	{
		if (node == nullptr)
			return;
		float dist = (target - this->Photons[node->idx].pos).squaredLength();
		if (dist < radius2)
			list.push_back(node->idx);
		float diff = target[node->axis] - this->Photons[node->idx].pos[node->axis];
		if (diff < 0)
			SearchNodeInRange(node->left, target, radius2, list);
		else
			SearchNodeInRange(node->right, target, radius2, list);
		if (radius2 > diff * diff)
		{
			if (diff < 0)
				SearchNodeInRange(node->right, target, radius2, list);
			else
				SearchNodeInRange(node->left, target, radius2, list);
		}
		
	}

	void ClearNode(Node* node)
	{
		if (node == nullptr)
			return;
		ClearNode(node->right);
		ClearNode(node->left);
		delete node;
	}

public:
	KdTree(){this->Photons = nullptr; this->num = 0; this->root = nullptr;}
	KdTree(Photon* photons, int n){ this->Photons = photons; this->num = n; this->root = nullptr; }
	~KdTree() { this->Clear(); }

	void Set(Photon* photons, int n){ this->Photons = photons; this->num = n; this->root = nullptr; }
	void Build()
	{
		std::vector<int> IdxArray(this->num);
		std::iota(IdxArray.begin(), IdxArray.end(), 0);
		this->root = this->KdNodeBuild(IdxArray.data(), this->num, 0);
	}

	float SearchKNN(const Vector3f& target, int k, std::vector<int>& result)
	{
		std::priority_queue<std::pair<float, int>> queue;
		this->SearchKNearestNode(this->root, target, k, queue);
		result.resize(queue.size());
		float max_dist = queue.top().first;
		for (int i = 0; i < queue.size(); i++)
		{
			result[i] = queue.top().second;
			queue.pop();
		}
		return max_dist;
	}

	int SearchNIR(const Vector3f& target, float radius2, std::vector<int>& result)
	{
		this->SearchNodeInRange(this->root, target, radius2, result);
		return result.size();
	}

	void Clear() { this->ClearNode(this->root); this->root = nullptr; }
};

class PhotonMap
{
private:
	std::vector<Photon> Photons;
	KdTree kdtree;

public:
	PhotonMap(){}
	PhotonMap(const std::vector<Photon>& photons) { this->Photons = photons; }

	void Set(const std::vector<Photon>& photons) { this->Photons = photons; }
	void push_back(const Photon& photon) { this->Photons.push_back(photon); }
	int GetSize() const { return this->Photons.size(); }
	Photon& operator[] (size_t i) { return this->Photons[i]; }
	void Clear() { this->Photons.clear(); this->kdtree.Clear(); }

	void Build()
	{
		this->kdtree.Clear();
		this->kdtree.Set(this->Photons.data(), this->Photons.size());
		this->kdtree.Build();
	}

	float QueryKNN(const Vector3f& target, int k, std::vector<int>& result)
	{
		return this->kdtree.SearchKNN(target, k, result);
	}

	float QueryNIR(const Vector3f& target, float radius2, std::vector<int>& result)
	{
		return this->kdtree.SearchNIR(target, radius2, result);
	}
};
#endif