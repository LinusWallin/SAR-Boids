using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Holds the parameters for a single simulation run.
/// </summary>
[System.Serializable]
public class SimulationConfig
{
    public string sceneName;
    public int numBoids;
    public int numLeaders;
    public bool useCBF;
    public bool useAPF;
    public bool usePath;
    public bool useMAPF;

    public override string ToString() =>
        $"{sceneName} | boids={numBoids} leaders={numLeaders} " +
        $"CBF={useCBF} APF={useAPF} Path={usePath} MAPF={useMAPF}";
}

/// <summary>
/// Builds a queue of SimulationConfigs and runs them one after another,
/// reloading the target scene between each run and applying settings
/// directly to the BoidSettings ScriptableObject before Start() fires.
///
/// Attach this to a persistent GameObject in your launcher/menu scene,
/// then set boidSettings in the Inspector.
/// </summary>
public class SimulationRunner : MonoBehaviour
{
    [Header("References")]
    [Tooltip("The shared BoidSettings asset that Simulation.cs reads from.")]
    public BoidSettings boidSettings;

    [Header("Runner config")]
    [Tooltip("How long (seconds) to wait after a scene loads before " +
             "checking whether Simulation has finished. Gives Start() " +
             "time to complete its setup.")]
    public float pollInterval = 0.5f;

    private Queue<SimulationConfig> queue = new Queue<SimulationConfig>();
    private SimulationConfig current;
    private int runIndex = 0;


    void Awake()
    {
        // Keep this object alive across scene loads.
        DontDestroyOnLoad(gameObject);
        BuildQueue();
    }

    void Start()
    {
        if (queue.Count == 0)
        {
            Debug.LogWarning("[SimulationRunner] Queue is empty – nothing to run.");
            return;
        }
        RunNext();
    }

    /// <summary>
    /// Enqueues a set of SimulationConfig runs for predefined scenes, boid counts, and configuration variants.
    /// </summary>
    private void BuildQueue()
    {
        string[] scenes =
        {
            "Corner",
            "Corridor",
            "Loop",
            "LoopWithCorridor",
            "Room",
            "RoomWithCorner"
        };

        int[] boidCounts = { 5, 10, 20 };

        foreach (var scene in scenes)
        {
            foreach (var count in boidCounts)
            {
                // --- Variant 1: baseline, no CBF ---
                queue.Enqueue(new SimulationConfig
                {
                    sceneName  = scene,
                    numBoids   = count,
                    numLeaders = 0,
                    useCBF     = false,
                    useAPF     = false,
                    usePath = false,
                    useMAPF = false
                });

                // --- Variant 2: CBF with all boids as leaders ---
                queue.Enqueue(new SimulationConfig
                {
                    sceneName  = scene,
                    numBoids   = count,
                    numLeaders = count,
                    useCBF     = true,
                    useAPF = false,
                    usePath = false,
                    useMAPF = false
                });

                // --- Variant 3: CBF + APF, no leaders ---
                queue.Enqueue(new SimulationConfig
                {
                    sceneName  = scene,
                    numBoids   = count,
                    numLeaders = 0,
                    useCBF     = true,
                    useAPF     = true,
                    usePath = false,
                    useMAPF = false
                });

                // --- Variant 4: CBF + Path + MAPF, no leaders ---
                queue.Enqueue(new SimulationConfig
                {
                    sceneName  = scene,
                    numBoids   = count,
                    numLeaders = 0,
                    useCBF     = true,
                    useAPF = false,
                    usePath    = true,
                    useMAPF    = true
                });
            }
        }

        Debug.Log($"[SimulationRunner] {queue.Count} runs queued " +
                  $"({scenes.Length} scenes × {boidCounts.Length} counts × 4 variants).");
    }

    /// <summary>
    /// Dequeues the next simulation, updates run state, applies its configuration, and loads the associated scene.
    /// </summary>
    private void RunNext()
    {
        if (queue.Count == 0)
        {
            Debug.Log("[SimulationRunner] All simulations complete.");
            return;
        }

        current = queue.Dequeue();
        runIndex++;

        Debug.Log($"[SimulationRunner] Starting run {runIndex}: {current}");

        ApplyConfig(current);

        SceneManager.sceneLoaded += OnSceneLoaded;
        SceneManager.LoadScene(current.sceneName);
    }

    /// <summary>
    /// Handles SceneManager.sceneLoaded by unsubscribing the handler and starting a coroutine to wait for the
    /// simulation to finish.
    /// </summary>
    /// <remarks>Unsubscribes SceneManager.sceneLoaded to prevent duplicate handling, then starts the
    /// WaitForSimulationToFinish coroutine.</remarks>
    /// <param name="scene">The scene that was loaded.</param>
    /// <param name="mode">The mode used to load the scene.</param>
    private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        StartCoroutine(WaitForSimulationToFinish());
    }

    /// <summary>
    /// Waits for the Simulation in the current scene to finish, polling at pollInterval, then advances the runner to
    /// the next run. If no Simulation is found, logs an error and advances immediately.
    /// </summary>
    /// <returns>An IEnumerator for use with Unity coroutines that yields until the simulation reports completion or the runner
    /// advances.</returns>
    private IEnumerator WaitForSimulationToFinish()
    {
        // Wait one frame for all Start() methods in the new scene to run.
        yield return null;

        Simulation sim = FindFirstObjectByType<Simulation>();
        if (sim == null)
        {
            Debug.LogError($"[SimulationRunner] No Simulation component found " +
                           $"in scene '{current.sceneName}'. Skipping.");
            RunNext();
            yield break;
        }

        // Poll until the Simulation marks itself finished.
        while (!sim.IsFinished)
        {
            yield return new WaitForSeconds(pollInterval);
        }

        Debug.Log($"[SimulationRunner] Run {runIndex} finished: {current}");
        RunNext();
    }

    /// <summary>
    /// Applies the provided SimulationConfig to the boid settings.
    /// </summary>
    /// <remarks>If boidSettings is null, an error is logged and the method returns without applying
    /// changes.</remarks>
    /// <param name="cfg">The SimulationConfig whose values are copied into the boid settings.</param>
    private void ApplyConfig(SimulationConfig cfg)
    {
        if (boidSettings == null)
        {
            Debug.LogError("[SimulationRunner] boidSettings is not assigned!");
            return;
        }

        boidSettings.numBoids      = cfg.numBoids;
        boidSettings.leaders       = cfg.numLeaders;
        boidSettings.isCBF         = cfg.useCBF;
        boidSettings.potentialField = cfg.useAPF;
        boidSettings.isPath        = cfg.usePath;
        boidSettings.isMAPF        = cfg.useMAPF;
    }
}
