using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class NodeNetworkCreator : MonoBehaviour {

	public int boardWidth = 100;
	public int boardHeight = 150;

	public IDictionary<Vector3, bool> walkablePositions = new Dictionary<Vector3, bool>();
	public IDictionary<Vector3, GameObject> nodeReference = new Dictionary<Vector3, GameObject>();
	public Dictionary<Vector3, string> obstacles = new Dictionary<Vector3, string>();

	// Use this for initialization
	void Start () {
		InitializeNodeNetwork (4000, 3500, 2000);
	}
	
	// Update is called once per frame
	void Update () {
	
	}

	void InitializeNodeNetwork(int numBarriers, int numSlow, int numVerySlow){
		
		var node = GameObject.Find ("Node");
		var obstacle = GameObject.Find ("Obstacle");
		var width = boardWidth;
		var height = boardHeight;

		obstacles = GenerateObstacles (numBarriers, numSlow, numVerySlow);

		Sprite slowTile = Resources.Load<Sprite> ("obstacle_slow 1");
		Sprite verySlowTile = Resources.Load<Sprite> ("obstacle_veryslow 1");

		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				Vector3 newPosition = new Vector3 (i, 0, j);
				GameObject copy;
				string obstacleType = null;

				if (obstacles.TryGetValue (newPosition, out obstacleType)) {
					copy = Instantiate (obstacle);
					copy.transform.position = newPosition;
					switch (obstacleType) {
					case "barrier":
						walkablePositions.Add (new KeyValuePair<Vector3, bool> (newPosition, false));
						break;
					case "slow":
						walkablePositions.Add (new KeyValuePair<Vector3, bool> (newPosition, true));
						copy.GetComponent<SpriteRenderer> ().sprite = slowTile;
						break;
					case "verySlow":
						walkablePositions.Add (new KeyValuePair<Vector3, bool> (newPosition, true));
						copy.GetComponent<SpriteRenderer> ().sprite = verySlowTile;
						break;
					}
				}
				else {
					copy = Instantiate (node);
					copy.transform.position = newPosition;
					walkablePositions.Add (new KeyValuePair<Vector3, bool> (newPosition, true));
				}

				nodeReference.Add (newPosition, copy);
			}
		}
        
        GameObject goal = GameObject.Find("Goal");
        walkablePositions[goal.transform.localPosition] = true;
        nodeReference[goal.transform.localPosition] = goal;
    }

	Dictionary<Vector3, string> GenerateObstacles(int numBarriers, int numSlow, int numVerySlow){

		for (int i = 0; i < numBarriers; i++) {
			Vector3 nodePosition = new Vector3 (Random.Range (0, boardWidth), 0, Random.Range (0, boardHeight));
			if(!obstacles.ContainsKey(nodePosition)){
				obstacles.Add(nodePosition, "barrier");
			}
		}

		for (int i = 0; i < numSlow; i++) {
			Vector3 nodePosition = new Vector3 (Random.Range (0, boardWidth), 0, Random.Range (0, boardHeight));
			if (!obstacles.ContainsKey (nodePosition)) {
				obstacles.Add(nodePosition, "slow");
			}
		}

		for (int i = 0; i < numSlow; i++) {
			Vector3 nodePosition = new Vector3 (Random.Range (0, boardWidth), 0, Random.Range (0, boardHeight));
			if (!obstacles.ContainsKey (nodePosition)) {
				obstacles.Add(nodePosition, "verySlow");
			}
		}

		return obstacles;
	}
}
