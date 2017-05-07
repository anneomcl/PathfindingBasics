using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Agent : MonoBehaviour {

	IDictionary<Vector3, Vector3> nodeParents = new Dictionary<Vector3, Vector3>();
	public IDictionary<Vector3, bool> walkablePositions;
	NodeNetworkCreator nodeNetwork;
	IList<Vector3> path;
	bool moveCube = false;
	int i;

	// Use this for initialization
	void Start () {
		nodeNetwork = GameObject.Find ("NodeNetwork").GetComponent<NodeNetworkCreator> ();
		walkablePositions = nodeNetwork.walkablePositions;
	}
	
	// Update is called once per frame
	void Update () {
		//Hacky way to move the cube along the path.
		if (moveCube) {
			float step = Time.deltaTime * 5;
			transform.position = Vector3.MoveTowards (transform.position, path[i], step);
			if (transform.position.Equals (path [i]) && i >= 0)
				i--;
			if (i < 0)
				moveCube = false;
		}
	}

	//Breadth first search of graph.
	//Populates IList<Vector3> path with a valid solution to the goalPosition.
	//Returns the goalPosition if a solution is found.
	//Returns the startPosition if no solution is found.
	Vector3 FindShortestPathBFS(Vector3 startPosition, Vector3 goalPosition){
		/*
			Put stuff in here.
		*/
		return startPosition;
	}

	//Depth first search of graph.
	//Populates IList<Vector3> path with a valid solution to the goalPosition.
	//Returns the goalPosition if a solution is found.
	//Returns the startPosition if no solution is found.
	Vector3 FindShortestPathDFS(Vector3 startPosition, Vector3 goalPosition){
		/*
			Put stuff in here.
		*/
		return startPosition;
	}

	bool CanMove(Vector3 nextPosition) {
		return (walkablePositions.ContainsKey (nextPosition) ? walkablePositions [nextPosition] : false);
	}

	public void DisplayShortestPath(bool isDFS) {
		
		nodeParents = new Dictionary<Vector3, Vector3>();
		path = FindShortestPath (isDFS);

		Sprite exploredTile = Resources.Load <Sprite>("path 1");
		Sprite victoryTile = Resources.Load<Sprite> ("victory 1");

		foreach (Vector3 node in path) {
			if (isDFS) {
				nodeNetwork.nodeReference [node].GetComponent<SpriteRenderer> ().sprite = victoryTile;
			} else {
				nodeNetwork.nodeReference [node].GetComponent<SpriteRenderer> ().sprite = exploredTile;
			}
		}

		nodeNetwork.nodeReference [path [0]].GetComponent<SpriteRenderer> ().sprite = victoryTile;

		i = path.Count - 1;
	}

	public void MoveCube(){
		moveCube = true;
	}

	IList<Vector3> FindShortestPath(bool isDFS = false){

		IList<Vector3> path = new List<Vector3> ();
		Vector3 goal;
		if (isDFS) {
			goal = FindShortestPathDFS (this.transform.localPosition, GameObject.Find ("Goal").transform.localPosition);
		} else {
			goal = FindShortestPathBFS (this.transform.localPosition, GameObject.Find ("Goal").transform.localPosition);
		}

		if (goal == this.transform.localPosition) {
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


	/*
	
	SOLUTIONS:



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
	
	
	
	 */
}
