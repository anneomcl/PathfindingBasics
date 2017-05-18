using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Priority_Queue;

public class Agent : MonoBehaviour {

	IDictionary<Vector3, Vector3> nodeParents = new Dictionary<Vector3, Vector3>();
	public IDictionary<Vector3, bool> walkablePositions;
	public IDictionary<Vector3, string> obstacles;
	IDictionary<Vector3, Sprite> prevSprite = new Dictionary<Vector3, Sprite> ();

	NodeNetworkCreator nodeNetwork;
	IList<Vector3> path;

	bool solutionVisible;
	string prevAlgo;

	bool moveCube = false;
	int i;

	// Use this for initialization
	void Start () {
		nodeNetwork = GameObject.Find ("NodeNetwork").GetComponent<NodeNetworkCreator> ();
		obstacles = GameObject.Find ("NodeNetwork").GetComponent<NodeNetworkCreator> ().obstacles;
		walkablePositions = nodeNetwork.walkablePositions;
	}
	
	// Update is called once per frame
	void Update () {
		//Hacky way to move the cube along the path.
		if (moveCube) {
            float speed = 5 / Weight(path[i]);
			float step = Time.deltaTime * speed;
			transform.position = Vector3.MoveTowards (transform.position, path[i], step);
			if (transform.position.Equals (path [i]) && i >= 0)
				i--;
			if (i < 0)
				moveCube = false;
		}
	}

    int EuclideanEstimate(Vector3 node, Vector3 goal)
    {
        return (int) Mathf.Sqrt(Mathf.Pow(node.x - goal.x, 2) +
            Mathf.Pow(node.y - goal.y, 2) +
            Mathf.Pow(node.z - goal.z, 2));
    }

    int ManhattanEstimate(Vector3 node, Vector3 goal)
    {
        return (int) (Mathf.Abs(node.x - goal.x) +
            Mathf.Abs(node.y - goal.y) +
            Mathf.Abs(node.z - goal.z));
    }

    int HeuristicCostEstimate(Vector3 node, Vector3 goal, string heuristic)
    {
        switch (heuristic)
        {
            case "euclidean":
                return EuclideanEstimate(node, goal);
            case "manhattan":
                return ManhattanEstimate(node, goal);
        }

        return -1;
    }

    Vector3 FindShortestPathAStar(Vector3 startPosition, Vector3 goalPosition, string heuristic) {

        // A* tries to minimize f(x) = g(x) + h(x), where g(x) is the distance from the start to node "x" and
        //    h(x) is some heuristic that must be admissible, meaning it never overestimates the cost to the next node.
        //    There are formal logical proofs you can look up that determine how heuristics are and are not admissible.

        IEnumerable<Vector3> validNodes = walkablePositions
            .Where(x => x.Value == true)
            .Select(x => x.Key);

        // Represents h(x) or the score from whatever heuristic we're using
        IDictionary<Vector3, int> heuristicScore = new Dictionary<Vector3, int>();

        // Represents g(x) or the distance from start to node "x" (Same meaning as in Dijkstra's "distances")
        IDictionary<Vector3, int> distanceFromStart = new Dictionary<Vector3, int>();

        foreach (Vector3 vertex in validNodes)
        {
            heuristicScore.Add(new KeyValuePair<Vector3, int>(vertex, int.MaxValue));
            distanceFromStart.Add(new KeyValuePair<Vector3, int>(vertex, int.MaxValue));
        }

        heuristicScore[startPosition] = HeuristicCostEstimate(startPosition, goalPosition, heuristic);
        distanceFromStart[startPosition] = 0;

        // Nodes we've already explored
        HashSet<Vector3> exploredNodes = new HashSet<Vector3>();

        // The item dequeued from a priority queue will always be the one with the lowest int value
        //    In this case we will input nodes with their calculated distances from the start g(x),
        //    so we will always take the node with the lowest distance from the queue.
        SimplePriorityQueue<Vector3, int> priorityQueue = new SimplePriorityQueue<Vector3, int>();
        priorityQueue.Enqueue(startPosition, distanceFromStart[startPosition]);

        while(priorityQueue.Count > 0)
        {
            // Get the node with the least distance from the start
            Vector3 curr = priorityQueue.Dequeue();

            // If our current node is the goal then stop
            if (curr == goalPosition)
            {
                print("A*" + heuristic + ": " + distanceFromStart[goalPosition]);
                return goalPosition;
            }

            //Otherwise, add it to our explored nodes set and look at its neighbors
            exploredNodes.Add(curr);

            IList<Vector3> neighbors = GetWalkableNodes(curr);

            foreach (Vector3 node in neighbors)
            {
                // If we've explored a neighbor, don't look at it again
                if (exploredNodes.Contains(node))
                {
                    continue;
                }

                // Get the distance so far, add it to the distance to the neighbor
                int currScore = distanceFromStart[curr] + Weight(node);

                // If we have not explored the neighbor, add it to the queue
                if (!priorityQueue.Contains(node))
                {
                    priorityQueue.Enqueue(node, distanceFromStart[node]);
                }
                // If our distance to this neighbor is MORE than another calculated shortest path
                //    to this neighbor, skip it.
                else if (currScore >= distanceFromStart[node])
                {
                    continue;
                }

                // Otherwise if the score is LESS, we will set a new node parent and update the scores
                //    as our current best for the path so far.
                nodeParents[node] = curr;
                distanceFromStart[node] = currScore;
                heuristicScore[node] = distanceFromStart[node] + HeuristicCostEstimate(node, goalPosition, heuristic);
            }
        }

        return startPosition;
    }

	//Dijkstra's algorithm.
	//Populates IList<Vector3> path with a valid solution to the goalPosition.
	//Returns the goalPosition if a solution is found.
	//Returns the startPosition if no solution is found.
	Vector3 FindShortestPathDijkstra(Vector3 startPosition, Vector3 goalPosition){

		HashSet<Vector3> unexploredNodes = new HashSet<Vector3> ();
        IDictionary<Vector3, int> distances = new Dictionary<Vector3, int>();
        IEnumerable<Vector3> validNodes = walkablePositions
			.Where (x => x.Value == true)
			.Select (x => x.Key);

		foreach (Vector3 vertex in validNodes) {
			distances.Add (new KeyValuePair<Vector3, int> (vertex, int.MaxValue));
			nodeParents.Add (new KeyValuePair<Vector3, Vector3> (vertex, new Vector3(-1, -1, -1)));
			unexploredNodes.Add (vertex);
		}

		distances [startPosition] = 0;

		while (unexploredNodes.Count > 0) {
			Vector3 curr = distances.Where(x => unexploredNodes.Contains(x.Key))
									.OrderBy (x => x.Value).First ().Key;
			if (curr == goalPosition)
            {
                print("Dijkstra: " + distances[goalPosition]);
                return goalPosition;
            }

			unexploredNodes.Remove (curr);

			IList<Vector3> nodes = GetWalkableNodes (curr);

			foreach (Vector3 node in nodes) {
				int dist = distances [curr] + Weight (node);
				if (dist < distances [node]) {
					distances [node] = dist;
					nodeParents [node] = curr;
				}
			}
		}

		return startPosition;
	}

	int Weight(Vector3 node) {
		if (obstacles.Keys.Contains(node)) {
			if (obstacles [node] == "slow") {
				return 3;
			} else if (obstacles [node] == "verySlow") {
				return 5;
			} else {
				return 1;
			}
		} else {
			return 1;
		}
	}

	//Breadth first search of graph.
	//Populates IList<Vector3> path with a valid solution to the goalPosition.
	//Returns the goalPosition if a solution is found.
	//Returns the startPosition if no solution is found.
	Vector3 FindShortestPathBFS(Vector3 startPosition, Vector3 goalPosition){
		Queue<Vector3> queue = new Queue<Vector3> ();
		HashSet<Vector3> exploredNodes = new HashSet<Vector3> ();
		queue.Enqueue (startPosition);

		while (queue.Count != 0) {
			Vector3 currentNode = queue.Dequeue ();
			if (currentNode == goalPosition) {
				return currentNode;
			}

			IList<Vector3> nodes = GetWalkableNodes (currentNode);

			foreach(Vector3 node in nodes){
				if(!exploredNodes.Contains(node)) {
					//Mark the node as explored
					exploredNodes.Add(node);

					//Store a reference to the previous node
					nodeParents.Add (node, currentNode);

					//Add this to the queue of nodes to examine
					queue.Enqueue (node);
				}
			}
		}

		return startPosition;
	}

	//Depth first search of graph.
	//Populates IList<Vector3> path with a valid solution to the goalPosition.
	//Returns the goalPosition if a solution is found.
	//Returns the startPosition if no solution is found.
	Vector3 FindShortestPathDFS(Vector3 startPosition, Vector3 goalPosition){
		Stack<Vector3> stack = new Stack<Vector3> ();
		HashSet<Vector3> exploredNodes = new HashSet<Vector3> ();
		stack.Push (startPosition);

		while (stack.Count != 0) {
			Vector3 currentNode = stack.Pop ();
			if (currentNode == goalPosition) {
				return currentNode;
			}

			IList<Vector3> nodes = GetWalkableNodes (currentNode);

			foreach(Vector3 node in nodes){
				if(!exploredNodes.Contains(node)) {
					//Mark the node as explored
					exploredNodes.Add(node);

					//Store a reference to the previous node
					nodeParents.Add (node, currentNode);

					//Add this to the queue of nodes to examine
					stack.Push (node);
				}
			}
		}

		return startPosition;
	}

	bool CanMove(Vector3 nextPosition) {
		return (walkablePositions.ContainsKey (nextPosition) ? walkablePositions [nextPosition] : false);
	}

	public void DisplayShortestPath(string algorithm) {

		if (solutionVisible && algorithm == prevAlgo) {
			foreach (Vector3 node in path) {
				nodeNetwork.nodeReference [node].GetComponent<SpriteRenderer> ().sprite = prevSprite[node];
			}

			solutionVisible = false;
			return;
		}
			
		nodeParents = new Dictionary<Vector3, Vector3>();
		path = FindShortestPath (algorithm);

		if (path == null)
			return;

		Sprite exploredTile = Resources.Load <Sprite>("path 1");
		Sprite victoryTile = Resources.Load<Sprite> ("victory 1");
		Sprite dijkstraTile = Resources.Load<Sprite> ("dijkstra");

		foreach (Vector3 node in path) {
			
			prevSprite[node] = nodeNetwork.nodeReference [node].GetComponent<SpriteRenderer> ().sprite;

            if (algorithm == "DFS")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = victoryTile;
            }
            else if (algorithm == "BFS")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = exploredTile;
            }
            else if (algorithm == "AStarEuclid")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = victoryTile;
            }
            else if (algorithm == "AStarManhattan")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = dijkstraTile;
            }
            else
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = dijkstraTile;
            }
		}

		nodeNetwork.nodeReference [path [0]].GetComponent<SpriteRenderer> ().sprite = victoryTile;

		i = path.Count - 1;

		solutionVisible = true;
		prevAlgo = algorithm;
	}

	public void MoveCube(){
		moveCube = true;
	}

	IList<Vector3> FindShortestPath(string algorithm){

		IList<Vector3> path = new List<Vector3> ();
		Vector3 goal;
        if (algorithm == "DFS")
        {
            goal = FindShortestPathDFS(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }
        else if (algorithm == "BFS")
        {
            goal = FindShortestPathBFS(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }
        else if (algorithm == "AStarEuclid")
        {
            goal = FindShortestPathAStar(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition, "euclidean");
        }
        else if (algorithm == "AStarManhattan")
        {
            goal = FindShortestPathAStar(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition, "manhattan");
        }
        else
        {
            goal = FindShortestPathDijkstra(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }

		if (goal == this.transform.localPosition || !nodeParents.ContainsKey(nodeParents[goal])) {
			//No solution was found.
			return null;
		}

		Vector3 curr = goal;
		while (curr != this.transform.localPosition) {
			path.Add (curr);
			curr = nodeParents [curr];
		}

		return path;
	}

	IList<Vector3> GetWalkableNodes(Vector3 curr) {

		IList<Vector3> walkableNodes = new List<Vector3> ();

		IList<Vector3> possibleNodes = new List<Vector3> () {
			new Vector3 (curr.x + 1, curr.y, curr.z),
			new Vector3 (curr.x - 1, curr.y, curr.z),
			new Vector3 (curr.x, curr.y, curr.z + 1),
			new Vector3 (curr.x, curr.y, curr.z - 1)
		};

		foreach (Vector3 node in possibleNodes) {
			if (CanMove (node)) {
				walkableNodes.Add (node);
			} 
		}

		return walkableNodes;
	}
}
