import {
  buildCircleTrajectory,
  buildMultiPointTrajectory,
  buildStudentMatrixFromRow,
  clamp,
  cloneJointVector,
  computePoseError,
  forwardKinematicsMDH,
  inferRowType,
  isPoseErrorPassing,
  jointVectorDegToRad,
  matrixFromFlatRowMajor,
  matrixToPoseMmDeg,
  parseNumericCsv,
  poseMmDegToMatrix,
  poseToViewerPoint,
  posesToViewerPoints,
  randomJointVectorDeg,
  radToDeg,
  round,
} from "./kinematics-lab-math.js";
import {
  DH_PRESET_CASES,
  FK_BATCH_CASES,
  STANDARD_MDH,
  VALIDATION_THRESHOLDS,
} from "./kinematics-lab-standard.js";

function createStatusBadge(pass) {
  return `<span class="lab-pass ${pass ? "pass" : "fail"}">${pass ? "Pass" : "Fail"}</span>`;
}

function formatPoseCard(pose) {
  return `
    <div class="lab-pose-grid">
      <div><span class="label">X</span><strong>${round(pose.x, 2)} mm</strong></div>
      <div><span class="label">Y</span><strong>${round(pose.y, 2)} mm</strong></div>
      <div><span class="label">Z</span><strong>${round(pose.z, 2)} mm</strong></div>
      <div><span class="label">Roll</span><strong>${round(pose.roll, 2)} deg</strong></div>
      <div><span class="label">Pitch</span><strong>${round(pose.pitch, 2)} deg</strong></div>
      <div><span class="label">Yaw</span><strong>${round(pose.yaw, 2)} deg</strong></div>
    </div>
  `;
}

export class KinematicsLab {
  constructor(container, { viewer, setStatus }) {
    this.container = container;
    this.viewer = viewer;
    this.setStatus = setStatus;
    this.kinematics = null;

    this.thresholds = VALIDATION_THRESHOLDS;
    this.currentPane = "dh";

    this.dhState = {
      inspectJointDeg: [0, 0, 0, 0, 0, 0],
      testCases: this.#buildDhCases(),
      lastValidation: [],
    };

    this.fkState = {
      jointDeg: [0, 0, 0, 0, 0, 0],
      batchJointSetsDeg: FK_BATCH_CASES.map((jointDeg) => cloneJointVector(jointDeg)),
    };

    this.ikState = {
      mode: "single",
      targetPose: { x: 420, y: 0, z: 240, roll: 90, pitch: 0, yaw: 0 },
      targetPath: [],
      importedJointRowsDeg: [],
      solutionIndex: 0,
      playbackTimer: null,
      playbackIndex: 0,
      speedMs: 250,
      lastResults: [],
    };

    this.#render();
    this.#cacheDom();
    this.#buildDynamicInputs();
    this.#bindEvents();
    this.#refreshDhInspect();
    this.#refreshFkComparison();
    this.#refreshIkTargets();
  }

  setRobotContext(kinematics) {
    this.kinematics = kinematics;
    this.#applyRobotJointPose(this.dhState.inspectJointDeg);
    this.#refreshFkComparison();
  }

  destroy() {
    this.#stopIkPlayback();
  }

  #render() {
    this.container.innerHTML = `
      <div class="lab-shell">
        <div class="card-title-row">
          <div>
            <h2>Kinematics Lab</h2>
            <div class="label">Offline teaching verification mode for standard-result comparison and 3D visualization only. No real hardware motion.</div>
          </div>
          <span class="badge badge-muted">preview-first</span>
        </div>

        <div class="lab-tab-row">
          <button type="button" class="lab-subtab active" data-lab-pane="dh">DH Check</button>
          <button type="button" class="lab-subtab" data-lab-pane="fk">Forward Kinematics</button>
          <button type="button" class="lab-subtab" data-lab-pane="ik">Inverse Kinematics</button>
        </div>

        <section class="lab-pane active" data-lab-pane="dh">
          <div class="lab-grid">
            <div class="lab-section">
              <h3>Student DH Input</h3>
              <div class="label">Enter the 6-axis parameter table for the current inspection pose. Units: alpha / theta in deg, a / d in mm.</div>
              <div id="labDhTable"></div>
              <div class="button-grid compact-gap">
                <button type="button" id="labValidateDhBtn">Validate Current DH</button>
                <button type="button" id="labRandomDhCaseBtn">Regenerate Test Cases</button>
              </div>
            </div>
            <div class="lab-section">
              <h3>Inspection and Results</h3>
              <div class="label">Blue marker: TCP from the student DH. URDF reference pose is only in the result panels (no green sphere).</div>
              <div id="labDhCaseButtons" class="lab-chip-row"></div>
              <div id="labDhInspectSliders" class="lab-joint-inputs"></div>
              <div id="labDhInspectSummary" class="lab-result-card"></div>
              <div id="labDhValidationSummary" class="lab-result-card"></div>
              <div id="labDhResults"></div>
            </div>
          </div>
        </section>

        <section class="lab-pane" data-lab-pane="fk">
          <div class="lab-grid">
            <div class="lab-section">
              <h3>Joint Angles and Student Matrix</h3>
              <div class="label">Joint angles use deg. Enter the student's 4x4 homogeneous matrix for the current pose.</div>
              <div id="labFkJointInputs" class="lab-joint-inputs"></div>
              <div class="lab-inline-row">
                <button type="button" id="labFkCompareBtn">Compare Student Result</button>
              </div>
              <div id="labFkMatrixInputs" class="lab-matrix-grid"></div>
            </div>
            <div class="lab-section">
              <h3>Reference Result and Batch Validation</h3>
              <div id="labFkStandardPose" class="lab-result-card"></div>
              <div id="labFkComparison" class="lab-result-card"></div>
              <div class="lab-inline-row">
                <button type="button" id="labFkRandomBatchBtn">Reload 5 Sample Joint Sets</button>
                <label class="inline-file">
                  <span>Import CSV</span>
                  <input id="labFkCsvInput" type="file" accept=".csv,.txt" />
                </label>
              </div>
              <div id="labFkBatchInfo" class="lab-result-card"></div>
              <div id="labFkBatchResults"></div>
            </div>
          </div>
        </section>

        <section class="lab-pane" data-lab-pane="ik">
          <div class="lab-grid">
            <div class="lab-section">
              <h3>Target Pose / Target Path</h3>
              <div class="label">Single-target mode supports multiple solutions. Path mode validates the imported joint path step by step.</div>
              <div class="lab-inline-row">
                <label>Target Mode
                  <select id="labIkTargetMode">
                    <option value="single">Single Target</option>
                    <option value="circle">Circular Path</option>
                    <option value="multipoint">Multi-Point Path</option>
                  </select>
                </label>
                <button type="button" id="labIkRefreshTargetBtn">Refresh Target</button>
              </div>
              <div id="labIkTargetInputs" class="lab-pose-inputs"></div>
              <div id="labIkTargetSummary" class="lab-result-card"></div>
              <div class="lab-inline-row">
                <label class="inline-file">
                  <span>Import Joint CSV</span>
                  <input id="labIkCsvInput" type="file" accept=".csv,.txt" />
                </label>
                <label>Playback Speed (ms)
                  <input id="labIkSpeedInput" type="number" min="50" max="2000" step="10" value="250" />
                </label>
              </div>
              <div class="button-grid compact-gap">
                <button type="button" id="labIkPlayBtn">Play</button>
                <button type="button" id="labIkPauseBtn">Pause</button>
                <button type="button" id="labIkStepBtn">Step</button>
                <button type="button" id="labIkResetBtn">Reset</button>
              </div>
            </div>
            <div class="lab-section">
              <h3>IK Validation Result</h3>
              <div class="lab-inline-row">
                <label>Solution Selector
                  <select id="labIkSolutionSelect"></select>
                </label>
              </div>
              <div id="labIkSummary" class="lab-result-card"></div>
              <div id="labIkResults"></div>
            </div>
          </div>
        </section>
      </div>
    `;
  }

  #cacheDom() {
    this.labTabButtons = [...this.container.querySelectorAll(".lab-subtab")];
    this.labPanes = [...this.container.querySelectorAll(".lab-pane")];

    this.dhTableEl = this.container.querySelector("#labDhTable");
    this.dhCaseButtonsEl = this.container.querySelector("#labDhCaseButtons");
    this.dhInspectSlidersEl = this.container.querySelector("#labDhInspectSliders");
    this.dhInspectSummaryEl = this.container.querySelector("#labDhInspectSummary");
    this.dhValidationSummaryEl = this.container.querySelector("#labDhValidationSummary");
    this.dhResultsEl = this.container.querySelector("#labDhResults");

    this.fkJointInputsEl = this.container.querySelector("#labFkJointInputs");
    this.fkMatrixInputsEl = this.container.querySelector("#labFkMatrixInputs");
    this.fkStandardPoseEl = this.container.querySelector("#labFkStandardPose");
    this.fkComparisonEl = this.container.querySelector("#labFkComparison");
    this.fkBatchInfoEl = this.container.querySelector("#labFkBatchInfo");
    this.fkBatchResultsEl = this.container.querySelector("#labFkBatchResults");
    this.fkCsvInputEl = this.container.querySelector("#labFkCsvInput");

    this.ikTargetModeEl = this.container.querySelector("#labIkTargetMode");
    this.ikTargetInputsEl = this.container.querySelector("#labIkTargetInputs");
    this.ikTargetSummaryEl = this.container.querySelector("#labIkTargetSummary");
    this.ikCsvInputEl = this.container.querySelector("#labIkCsvInput");
    this.ikSpeedInputEl = this.container.querySelector("#labIkSpeedInput");
    this.ikSolutionSelectEl = this.container.querySelector("#labIkSolutionSelect");
    this.ikSummaryEl = this.container.querySelector("#labIkSummary");
    this.ikResultsEl = this.container.querySelector("#labIkResults");
  }

  #buildDynamicInputs() {
    this.#buildDhTable();
    this.#buildDhCaseButtons();
    this.#buildJointInputs(this.dhInspectSlidersEl, "dhInspect", this.dhState.inspectJointDeg, () =>
      this.#refreshDhInspect()
    );

    this.#buildJointInputs(this.fkJointInputsEl, "fkJoint", this.fkState.jointDeg, () =>
      this.#refreshFkComparison()
    );
    this.#buildFkMatrixInputs();
    this.#buildIkTargetInputs();
    this.#renderFkBatchInfo();
    this.#renderIkSolutionSelector();
  }

  #bindEvents() {
    this.labTabButtons.forEach((button) => {
      button.addEventListener("click", () => {
        this.currentPane = button.dataset.labPane;
        this.labTabButtons.forEach((tab) =>
          tab.classList.toggle("active", tab === button)
        );
        this.labPanes.forEach((pane) =>
          pane.classList.toggle("active", pane.dataset.labPane === this.currentPane)
        );
        this.#refreshCurrentPaneVisualization();
      });
    });

    this.container.querySelector("#labValidateDhBtn").addEventListener("click", () =>
      this.#validateDh()
    );
    this.container.querySelector("#labRandomDhCaseBtn").addEventListener("click", () => {
      this.dhState.testCases = this.#buildDhCases();
      this.#buildDhCaseButtons();
      this.#refreshDhInspect();
      this.setStatus("DH test cases regenerated.", "ok");
    });

    this.container.querySelector("#labFkCompareBtn").addEventListener("click", () =>
      this.#refreshFkComparison()
    );
    this.container.querySelector("#labFkRandomBatchBtn").addEventListener("click", () => {
      this.fkState.batchJointSetsDeg = FK_BATCH_CASES.map((jointDeg) => cloneJointVector(jointDeg));
      this.#renderFkBatchInfo();
      this.fkBatchResultsEl.innerHTML = "";
      this.setStatus("Reloaded the 5 FK teaching samples.", "ok");
    });
    this.fkCsvInputEl.addEventListener("change", async (event) => {
      const file = event.target.files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        this.#runFkBatchComparison(text);
        this.setStatus("FK CSV batch validation completed.", "ok");
      } catch (error) {
        this.fkBatchResultsEl.innerHTML = `<div class="lab-error">${error.message}</div>`;
        this.setStatus(error.message, "danger-text");
      } finally {
        event.target.value = "";
      }
    });

    this.ikTargetModeEl.addEventListener("change", () => {
      this.ikState.mode = this.ikTargetModeEl.value;
      this.#refreshIkTargets();
    });
    this.container.querySelector("#labIkRefreshTargetBtn").addEventListener("click", () =>
      this.#refreshIkTargets()
    );

    this.ikCsvInputEl.addEventListener("change", async (event) => {
      const file = event.target.files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        this.#loadIkCsv(text);
        this.setStatus("IK student solutions imported and validated.", "ok");
      } catch (error) {
        this.ikSummaryEl.innerHTML = `<div class="lab-error">${error.message}</div>`;
        this.setStatus(error.message, "danger-text");
      } finally {
        event.target.value = "";
      }
    });

    this.ikSpeedInputEl.addEventListener("change", () => {
      this.ikState.speedMs = clamp(Number(this.ikSpeedInputEl.value) || 250, 50, 2000);
      this.ikSpeedInputEl.value = String(this.ikState.speedMs);
      if (this.ikState.playbackTimer) {
        this.#stopIkPlayback();
        this.#startIkPlayback();
      }
    });

    this.ikSolutionSelectEl.addEventListener("change", () => {
      this.ikState.solutionIndex = Number(this.ikSolutionSelectEl.value) || 0;
      this.#refreshIkResults();
    });

    this.container.querySelector("#labIkPlayBtn").addEventListener("click", () =>
      this.#startIkPlayback()
    );
    this.container.querySelector("#labIkPauseBtn").addEventListener("click", () =>
      this.#stopIkPlayback()
    );
    this.container.querySelector("#labIkStepBtn").addEventListener("click", () =>
      this.#stepIkPlayback()
    );
    this.container.querySelector("#labIkResetBtn").addEventListener("click", () => {
      this.#stopIkPlayback();
      this.ikState.playbackIndex = 0;
      this.#refreshIkResults();
    });
  }

  #buildDhCases() {
    const randomCase = {
      id: "random",
      label: "Random Pose",
      jointDeg: randomJointVectorDeg(STANDARD_MDH.limitsRad),
    };
    return [...DH_PRESET_CASES, randomCase];
  }

  #buildDhTable() {
    const rows = Array.from({ length: 6 }, (_, index) => `
      <div class="lab-dh-row">
        <div class="lab-dh-cell">J${index + 1}</div>
        <input type="number" step="0.01" data-dh-row="${index}" data-dh-key="alpha" placeholder="alpha (deg)" />
        <input type="number" step="0.01" data-dh-row="${index}" data-dh-key="a" placeholder="a (mm)" />
        <input type="number" step="0.01" data-dh-row="${index}" data-dh-key="d" placeholder="d (mm)" />
        <input type="number" step="0.01" data-dh-row="${index}" data-dh-key="theta" placeholder="theta (deg)" />
      </div>
    `).join("");

    this.dhTableEl.innerHTML = `
      <div class="lab-dh-header">
        <div>Link</div>
        <div>alpha_(i-1)</div>
        <div>a_(i-1)</div>
        <div>d_i</div>
        <div>theta_i (current pose)</div>
      </div>
      ${rows}
    `;
  }

  #buildDhCaseButtons() {
    this.dhCaseButtonsEl.innerHTML = this.dhState.testCases
      .map(
        (item, index) =>
          `<button type="button" class="lab-chip" data-case-index="${index}">${item.label}</button>`
      )
      .join("");

    [...this.dhCaseButtonsEl.querySelectorAll(".lab-chip")].forEach((button) => {
      button.addEventListener("click", () => {
        const caseIndex = Number(button.dataset.caseIndex) || 0;
        this.dhState.inspectJointDeg = cloneJointVector(
          this.dhState.testCases[caseIndex].jointDeg
        );
        this.#setJointInputValues("dhInspect", this.dhState.inspectJointDeg);
        this.#refreshDhInspect();
      });
    });
  }

  #buildJointInputs(container, prefix, stateRef, onChange) {
    const rows = STANDARD_MDH.limitsRad.min
      .map((minValue, index) => {
        const minDeg = radToDeg(minValue);
        const maxDeg = radToDeg(STANDARD_MDH.limitsRad.max[index]);
        const value = stateRef[index] || 0;
        return `
          <div class="lab-joint-row">
            <div class="lab-joint-name">J${index + 1}</div>
            <input id="${prefix}Range${index}" type="range" min="${round(minDeg, 2)}" max="${round(
              maxDeg,
              2
            )}" step="0.1" value="${value}" />
            <input id="${prefix}Number${index}" type="number" min="${round(minDeg, 2)}" max="${round(
              maxDeg,
              2
            )}" step="0.1" value="${round(value, 2)}" />
            <div class="label">${round(minDeg, 1)} ~ ${round(maxDeg, 1)} deg</div>
          </div>
        `;
      })
      .join("");

    container.innerHTML = rows;

    STANDARD_MDH.limitsRad.min.forEach((_, index) => {
      const rangeEl = container.querySelector(`#${prefix}Range${index}`);
      const numberEl = container.querySelector(`#${prefix}Number${index}`);
      const sync = (value) => {
        const numeric = Number(value) || 0;
        stateRef[index] = numeric;
        rangeEl.value = String(numeric);
        numberEl.value = round(numeric, 2);
        onChange();
      };

      rangeEl.addEventListener("input", (event) => sync(event.target.value));
      numberEl.addEventListener("change", (event) => sync(event.target.value));
    });
  }

  #setJointInputValues(prefix, values) {
    values.forEach((value, index) => {
      const rangeEl = this.container.querySelector(`#${prefix}Range${index}`);
      const numberEl = this.container.querySelector(`#${prefix}Number${index}`);
      if (rangeEl) rangeEl.value = String(value);
      if (numberEl) numberEl.value = round(value, 2);
    });
  }

  #buildFkMatrixInputs() {
    const defaultValues = [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ];

    this.fkMatrixInputsEl.innerHTML = defaultValues
      .map(
        (value, index) =>
          `<input type="number" step="0.001" data-fk-matrix="${index}" value="${value}" />`
      )
      .join("");

    [...this.fkMatrixInputsEl.querySelectorAll("input")].forEach((input) =>
      input.addEventListener("input", () => this.#refreshFkComparison())
    );
  }

  #buildIkTargetInputs() {
    const fields = [
      ["x", "X (mm)"],
      ["y", "Y (mm)"],
      ["z", "Z (mm)"],
      ["roll", "Roll (deg)"],
      ["pitch", "Pitch (deg)"],
      ["yaw", "Yaw (deg)"],
    ];

    this.ikTargetInputsEl.innerHTML = fields
      .map(
        ([key, label]) => `
          <label>${label}
            <input type="number" step="0.01" data-ik-target="${key}" value="${this.ikState.targetPose[key]}" />
          </label>
        `
      )
      .join("");

    [...this.ikTargetInputsEl.querySelectorAll("input")].forEach((input) =>
      input.addEventListener("change", () => this.#refreshIkTargets())
    );
  }

  #readDhInput(referenceJointDeg = null) {
    const keys = ["alpha", "a", "d", "theta"];
    const values = Array.from({ length: 6 }, () => ({}));

    [...this.dhTableEl.querySelectorAll("input")].forEach((input) => {
      const row = Number(input.dataset.dhRow);
      const key = input.dataset.dhKey;
      values[row][key] = Number.parseFloat(input.value);
    });

    const hasInvalid = values.some((row) => keys.some((key) => !Number.isFinite(row[key])));
    if (hasInvalid) {
      throw new Error("Please complete all 6 DH rows before validation.");
    }

    return {
      aMm: values.map((row) => row.a),
      alphaRad: values.map((row) => (row.alpha * Math.PI) / 180),
      dMm: values.map((row) => row.d),
      offsetRad: values.map((row, index) => {
        const thetaRad = (row.theta * Math.PI) / 180;
        const referenceJointRad = referenceJointDeg
          ? ((referenceJointDeg[index] || 0) * Math.PI) / 180
          : 0;
        return thetaRad - referenceJointRad;
      }),
      limitsRad: STANDARD_MDH.limitsRad,
    };
  }

  #readFkManualMatrix() {
    const values = [...this.fkMatrixInputsEl.querySelectorAll("input")].map((input) =>
      Number.parseFloat(input.value)
    );
    if (values.some((value) => !Number.isFinite(value))) {
      throw new Error("Student matrix input contains invalid values.");
    }
    return matrixFromFlatRowMajor(values);
  }

  #getUrdfFkReference(jointDeg) {
    if (!this.kinematics) {
      throw new Error("URDF robot model is not ready yet.");
    }

    this.#applyRobotJointPose(jointDeg);
    let matrixMm;

    if (typeof this.kinematics.getEndEffectorMatrix === "function") {
      const matrix = this.kinematics.getEndEffectorMatrix();
      matrixMm = matrix.clone();
      matrixMm.elements[12] *= 1000;
      matrixMm.elements[13] *= 1000;
      matrixMm.elements[14] *= 1000;
    } else if (typeof this.kinematics.getEndEffectorPose === "function") {
      const pose = this.kinematics.getEndEffectorPose();
      matrixMm = poseMmDegToMatrix({
        x: (pose.x || 0) * 1000,
        y: (pose.y || 0) * 1000,
        z: (pose.z || 0) * 1000,
        roll: radToDeg(pose.rx || 0),
        pitch: radToDeg(pose.ry || 0),
        yaw: radToDeg(pose.rz || 0),
      });
    } else {
      throw new Error("URDF end-effector pose API is unavailable.");
    }

    return {
      matrix: matrixMm,
      pose: matrixToPoseMmDeg(matrixMm),
    };
  }

  #refreshDhInspect() {
    if (!this.kinematics) {
      this.dhInspectSummaryEl.innerHTML = `
        <div class="lab-info">Waiting for the URDF robot model to become ready.</div>
      `;
      this.viewer.clearLabVisualization?.();
      return;
    }

    const studentMdh = this.#tryReadDhForInspect();
    const jointRad = jointVectorDegToRad(this.dhState.inspectJointDeg);
    const standard = this.#getUrdfFkReference(this.dhState.inspectJointDeg);

    this.#applyRobotJointPose(this.dhState.inspectJointDeg);

    if (!studentMdh) {
      this.dhInspectSummaryEl.innerHTML = `
        <div class="lab-info">Fill in the full DH table for the current inspection pose to compare the student result with the URDF end-effector reference.</div>
      `;
      this.viewer.clearLabVisualization?.();
      return;
    }

    const student = forwardKinematicsMDH(studentMdh, jointRad);
    const error = computePoseError(student.matrix, standard.matrix);
    const pass = isPoseErrorPassing(error, this.thresholds);

    this.dhInspectSummaryEl.innerHTML = `
      <div class="lab-summary-top">
        <strong>Current Inspection Pose</strong>
        ${createStatusBadge(pass)}
      </div>
      <div class="lab-error-grid">
        <div>Δx: <strong>${round(error.dx, 2)} mm</strong></div>
        <div>Δy: <strong>${round(error.dy, 2)} mm</strong></div>
        <div>Δz: <strong>${round(error.dz, 2)} mm</strong></div>
        <div>Position Norm: <strong>${round(error.positionNormMm, 3)} mm</strong></div>
        <div>Δroll: <strong>${round(error.dRoll, 2)} deg</strong></div>
        <div>Δpitch: <strong>${round(error.dPitch, 2)} deg</strong></div>
        <div>Δyaw: <strong>${round(error.dYaw, 2)} deg</strong></div>
        <div>Orientation Norm: <strong>${round(error.orientationNormDeg, 3)} deg</strong></div>
      </div>
    `;

    this.viewer.setLabMarkers?.({
      student: poseToViewerPoint(student.pose),
    });
    this.viewer.setLabTrajectories?.({ target: [], actual: [] });
  }

  #tryReadDhForInspect() {
    try {
      return this.#readDhInput(this.dhState.inspectJointDeg);
    } catch {
      return null;
    }
  }

  #validateDh() {
    try {
      const studentMdh = this.#readDhInput(this.dhState.inspectJointDeg);
      const rows = this.dhState.testCases.map((testCase) => {
        const jointRad = jointVectorDegToRad(testCase.jointDeg);
        const standard = this.#getUrdfFkReference(testCase.jointDeg);
        const student = forwardKinematicsMDH(studentMdh, jointRad);
        const error = computePoseError(student.matrix, standard.matrix);
        return {
          id: testCase.id,
          label: testCase.label,
          jointDeg: testCase.jointDeg,
          error,
          pass: isPoseErrorPassing(error, this.thresholds),
        };
      });

      this.dhState.lastValidation = rows;
      const allPass = rows.every((row) => row.pass);
      this.dhValidationSummaryEl.innerHTML = `
        <div class="lab-summary-top">
          <strong>${allPass ? "DH Validation Passed" : "Error Too Large - Please Check the DH Table"}</strong>
          ${createStatusBadge(allPass)}
        </div>
        <div class="label">The theta column is interpreted at the current inspection pose, then converted internally to offsets. Student DH is compared against the current URDF end-effector reference. Thresholds: position error &lt; ${this.thresholds.positionMm} mm, orientation error &lt; ${this.thresholds.orientationDeg} deg.</div>
      `;

      this.dhResultsEl.innerHTML = `
        <table class="lab-table">
          <thead>
            <tr>
              <th>Case</th>
              <th>Joint Angles (deg)</th>
              <th>Position Norm</th>
              <th>Orientation Norm</th>
              <th>Δx / Δy / Δz</th>
              <th>Δr / Δp / Δy</th>
              <th>Status</th>
            </tr>
          </thead>
          <tbody>
            ${rows
              .map(
                (row) => `
                  <tr>
                    <td>${row.label}</td>
                    <td>${row.jointDeg.map((value) => round(value, 1)).join(", ")}</td>
                    <td>${round(row.error.positionNormMm, 3)} mm</td>
                    <td>${round(row.error.orientationNormDeg, 3)} deg</td>
                    <td>${round(row.error.dx, 2)}, ${round(row.error.dy, 2)}, ${round(
                      row.error.dz,
                      2
                    )}</td>
                    <td>${round(row.error.dRoll, 2)}, ${round(row.error.dPitch, 2)}, ${round(
                      row.error.dYaw,
                      2
                    )}</td>
                    <td>${createStatusBadge(row.pass)}</td>
                  </tr>
                `
              )
              .join("")}
          </tbody>
        </table>
      `;

      this.#refreshDhInspect();
      this.setStatus(allPass ? "DH validation passed." : "DH validation failed.", allPass ? "ok" : "warn");
    } catch (error) {
      this.dhValidationSummaryEl.innerHTML = `<div class="lab-error">${error.message}</div>`;
      this.setStatus(error.message, "danger-text");
    }
  }

  #refreshFkComparison() {
    if (!this.kinematics) {
      this.fkStandardPoseEl.innerHTML = `<div class="lab-info">Waiting for the URDF robot model to become ready.</div>`;
      this.fkComparisonEl.innerHTML = `<div class="lab-info">Enter a 4x4 matrix after the robot preview finishes loading.</div>`;
      this.viewer.clearLabVisualization?.();
      return;
    }

    try {
      const standard = this.#getUrdfFkReference(this.fkState.jointDeg);
      const studentMatrix = this.#readFkManualMatrix();
      const error = computePoseError(studentMatrix, standard.matrix);
      const pass = isPoseErrorPassing(error, this.thresholds);
      const studentPose = matrixToPoseMmDeg(studentMatrix);

      this.fkStandardPoseEl.innerHTML = `
        <div class="lab-summary-top"><strong>Reference End-Effector Pose (URDF)</strong></div>
        ${formatPoseCard(standard.pose)}
      `;

      this.fkComparisonEl.innerHTML = `
        <div class="lab-summary-top">
          <strong>${pass ? "Forward Kinematics Correct" : "Student Result Differs from the Reference Result"}</strong>
          ${createStatusBadge(pass)}
        </div>
        <div class="lab-two-column">
          <div>
            <div class="label">Student Pose</div>
            ${formatPoseCard(studentPose)}
          </div>
          <div>
            <div class="label">Error</div>
            <div class="lab-error-grid">
              <div>Δx: <strong>${round(error.dx, 2)} mm</strong></div>
              <div>Δy: <strong>${round(error.dy, 2)} mm</strong></div>
              <div>Δz: <strong>${round(error.dz, 2)} mm</strong></div>
              <div>Position Norm: <strong>${round(error.positionNormMm, 3)} mm</strong></div>
              <div>Δroll: <strong>${round(error.dRoll, 2)} deg</strong></div>
              <div>Δpitch: <strong>${round(error.dPitch, 2)} deg</strong></div>
              <div>Δyaw: <strong>${round(error.dYaw, 2)} deg</strong></div>
              <div>Orientation Norm: <strong>${round(error.orientationNormDeg, 3)} deg</strong></div>
            </div>
          </div>
        </div>
      `;

      this.viewer.setLabMarkers?.({
        student: poseToViewerPoint(studentPose),
      });
      this.viewer.setLabTrajectories?.({ target: [], actual: [] });
    } catch (error) {
      this.fkComparisonEl.innerHTML = `<div class="lab-error">${error.message}</div>`;
      this.viewer.clearLabVisualization?.();
    }
  }

  #renderFkBatchInfo() {
    this.fkBatchInfoEl.innerHTML = `
      <div class="lab-summary-top"><strong>Current Batch Joint Samples</strong></div>
      <table class="lab-table">
        <thead>
          <tr>
            <th>#</th>
            <th>J1~J6 (deg)</th>
          </tr>
        </thead>
        <tbody>
          ${this.fkState.batchJointSetsDeg
            .map(
              (jointDeg, index) => `
                <tr>
                  <td>${index + 1}</td>
                  <td>${jointDeg.map((value) => round(value, 1)).join(", ")}</td>
                </tr>
              `
            )
            .join("")}
        </tbody>
      </table>
      <div class="label">Each CSV row must match one sample and contain 16 values for a 4x4 matrix in row-major order.</div>
    `;
  }

  #runFkBatchComparison(text) {
    const rows = parseNumericCsv(text);
    if (rows.length !== this.fkState.batchJointSetsDeg.length) {
      throw new Error(
        `CSV row count ${rows.length} does not match the current sample count ${this.fkState.batchJointSetsDeg.length}.`
      );
    }
    if (rows.some((row) => row.length !== 16)) {
      throw new Error("Each FK CSV row must contain 16 values for a 4x4 matrix.");
    }

    const results = rows.map((row, index) => {
      const studentMatrix = buildStudentMatrixFromRow(row);
      const standard = this.#getUrdfFkReference(this.fkState.batchJointSetsDeg[index]);
      const error = computePoseError(studentMatrix, standard.matrix);
      return {
        index: index + 1,
        rowType: inferRowType(row),
        error,
        pass: isPoseErrorPassing(error, this.thresholds),
      };
    });

    const passCount = results.filter((item) => item.pass).length;
    const avgPosition =
      results.reduce((sum, item) => sum + item.error.positionNormMm, 0) / results.length;
    const avgOrientation =
      results.reduce((sum, item) => sum + item.error.orientationNormDeg, 0) / results.length;

    this.fkBatchResultsEl.innerHTML = `
      <div class="lab-result-card">
        <div class="lab-summary-top">
          <strong>Batch Validation Complete</strong>
          ${createStatusBadge(passCount === results.length)}
        </div>
        <div class="lab-error-grid">
          <div>Pass Rate: <strong>${round((passCount / results.length) * 100, 1)}%</strong></div>
          <div>Average Position Error: <strong>${round(avgPosition, 3)} mm</strong></div>
          <div>Average Orientation Error: <strong>${round(avgOrientation, 3)} deg</strong></div>
        </div>
      </div>
      <table class="lab-table">
        <thead>
          <tr>
            <th>#</th>
            <th>Type</th>
            <th>Position Error</th>
            <th>Orientation Error</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody>
          ${results
            .map(
              (item) => `
                <tr>
                  <td>${item.index}</td>
                  <td>${item.rowType === "matrix" ? "Matrix" : "Pose"}</td>
                  <td>${round(item.error.positionNormMm, 3)} mm</td>
                  <td>${round(item.error.orientationNormDeg, 3)} deg</td>
                  <td>${createStatusBadge(item.pass)}</td>
                </tr>
              `
            )
            .join("")}
        </tbody>
      </table>
    `;
  }

  #refreshIkTargets() {
    this.#stopIkPlayback();

    if (this.ikState.mode === "single") {
      const pose = {};
      [...this.ikTargetInputsEl.querySelectorAll("input")].forEach((input) => {
        pose[input.dataset.ikTarget] = Number.parseFloat(input.value);
      });
      this.ikState.targetPose = pose;
      this.ikState.targetPath = [pose];
      this.ikTargetInputsEl.style.display = "grid";
    } else if (this.ikState.mode === "circle") {
      this.ikState.targetPath = buildCircleTrajectory();
      this.ikTargetInputsEl.style.display = "none";
    } else {
      this.ikState.targetPath = buildMultiPointTrajectory();
      this.ikTargetInputsEl.style.display = "none";
    }

    const label =
      this.ikState.mode === "single"
        ? "Single Target"
        : this.ikState.mode === "circle"
          ? "Circular Path"
          : "Multi-Point Path";

    this.ikTargetSummaryEl.innerHTML = `
      <div class="lab-summary-top"><strong>${label}</strong></div>
      <div class="label">Target Points: ${this.ikState.targetPath.length}</div>
      ${formatPoseCard(this.ikState.targetPath[0])}
    `;

    this.#refreshIkResults();
  }

  #loadIkCsv(text) {
    const rows = parseNumericCsv(text);
    if (rows.some((row) => row.length !== 6)) {
      throw new Error("Each IK CSV row must contain 6 joint angles in deg.");
    }

    this.ikState.importedJointRowsDeg = rows.map((row) => cloneJointVector(row));
    this.ikState.solutionIndex = 0;
    this.ikState.playbackIndex = 0;
    this.#renderIkSolutionSelector();
    this.#refreshIkResults();
  }

  #renderIkSolutionSelector() {
    if (this.ikState.mode !== "single" || !this.ikState.importedJointRowsDeg.length) {
      this.ikSolutionSelectEl.innerHTML = `<option value="0">None</option>`;
      this.ikSolutionSelectEl.disabled = true;
      return;
    }

    this.ikSolutionSelectEl.disabled = false;
    this.ikSolutionSelectEl.innerHTML = this.ikState.importedJointRowsDeg
      .map(
        (_, index) =>
          `<option value="${index}" ${index === this.ikState.solutionIndex ? "selected" : ""}>Solution ${index + 1}</option>`
      )
      .join("");
  }

  #refreshIkResults() {
    if (!this.kinematics) {
      this.ikSummaryEl.innerHTML = `<div class="lab-info">Waiting for the URDF robot model to become ready.</div>`;
      this.ikResultsEl.innerHTML = "";
      this.viewer.clearLabVisualization?.();
      return;
    }

    if (!this.ikState.importedJointRowsDeg.length) {
      this.ikSummaryEl.innerHTML = `<div class="lab-info">Import a student joint CSV to show error statistics and animation controls here.</div>`;
      this.ikResultsEl.innerHTML = "";
      this.viewer.clearLabVisualization?.();
      return;
    }

    if (this.ikState.mode === "single") {
      const targetMatrix = poseMmDegToMatrix(this.ikState.targetPath[0]);
      const solutions = this.ikState.importedJointRowsDeg.map((jointDeg, index) => {
        const actual = this.#getUrdfFkReference(jointDeg);
        const error = computePoseError(actual.matrix, targetMatrix);
        return {
          index,
          jointDeg,
          actual,
          error,
          pass: isPoseErrorPassing(error, this.thresholds),
        };
      });

      this.ikState.lastResults = solutions;
      this.#renderIkSolutionSelector();
      const active = solutions[this.ikState.solutionIndex] || solutions[0];

      this.ikSummaryEl.innerHTML = `
        <div class="lab-summary-top">
          <strong>${active.pass ? "Inverse Kinematics Correct" : "Inverse Kinematics Has Noticeable Error"}</strong>
          ${createStatusBadge(active.pass)}
        </div>
        <div class="lab-error-grid">
          <div>Position Error: <strong>${round(active.error.positionNormMm, 3)} mm</strong></div>
          <div>Orientation Error: <strong>${round(active.error.orientationNormDeg, 3)} deg</strong></div>
        </div>
      `;

      this.ikResultsEl.innerHTML = `
        <table class="lab-table">
          <thead>
            <tr>
              <th>Solution</th>
              <th>Joint Angles (deg)</th>
              <th>Position Error</th>
              <th>Orientation Error</th>
              <th>Status</th>
            </tr>
          </thead>
          <tbody>
            ${solutions
              .map(
                (item) => `
                  <tr>
                    <td>${item.index + 1}</td>
                    <td>${item.jointDeg.map((value) => round(value, 1)).join(", ")}</td>
                    <td>${round(item.error.positionNormMm, 3)} mm</td>
                    <td>${round(item.error.orientationNormDeg, 3)} deg</td>
                    <td>${createStatusBadge(item.pass)}</td>
                  </tr>
                `
              )
              .join("")}
          </tbody>
        </table>
      `;

      this.#applyRobotJointPose(active.jointDeg);
      this.viewer.setLabMarkers?.({
        actual: this.kinematics.getEndEffectorPositionMeters(),
      });
      this.viewer.setLabTrajectories?.({ target: [], actual: [] });
      return;
    }

    if (this.ikState.importedJointRowsDeg.length !== this.ikState.targetPath.length) {
      this.ikSummaryEl.innerHTML = `
        <div class="lab-error">The target path has ${this.ikState.targetPath.length} points, but the imported CSV contains ${this.ikState.importedJointRowsDeg.length} rows.</div>
      `;
      this.ikResultsEl.innerHTML = "";
      this.viewer.clearLabVisualization?.();
      return;
    }

    const results = this.ikState.targetPath.map((targetPose, index) => {
      const targetMatrix = poseMmDegToMatrix(targetPose);
      const actual = this.#getUrdfFkReference(this.ikState.importedJointRowsDeg[index]);
      const error = computePoseError(actual.matrix, targetMatrix);
      return {
        index,
        targetPose,
        actual,
        error,
        pass: isPoseErrorPassing(error, this.thresholds),
      };
    });

    this.ikState.lastResults = results;
    const passCount = results.filter((item) => item.pass).length;
    const maxPosition = Math.max(...results.map((item) => item.error.positionNormMm));
    const avgPosition =
      results.reduce((sum, item) => sum + item.error.positionNormMm, 0) / results.length;

    this.ikSummaryEl.innerHTML = `
      <div class="lab-summary-top">
        <strong>${passCount === results.length ? "Inverse Kinematics Correct" : "Path Error Exceeds the Threshold"}</strong>
        ${createStatusBadge(passCount === results.length)}
      </div>
      <div class="lab-error-grid">
        <div>Pass Rate: <strong>${round((passCount / results.length) * 100, 1)}%</strong></div>
        <div>Max Position Error: <strong>${round(maxPosition, 3)} mm</strong></div>
        <div>Average Position Error: <strong>${round(avgPosition, 3)} mm</strong></div>
      </div>
    `;

    this.ikResultsEl.innerHTML = `
      <table class="lab-table">
        <thead>
          <tr>
            <th>#</th>
            <th>Position Error</th>
            <th>Orientation Error</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody>
          ${results
            .map(
              (item) => `
                <tr>
                  <td>${item.index + 1}</td>
                  <td>${round(item.error.positionNormMm, 3)} mm</td>
                  <td>${round(item.error.orientationNormDeg, 3)} deg</td>
                  <td>${createStatusBadge(item.pass)}</td>
                </tr>
              `
            )
            .join("")}
        </tbody>
      </table>
    `;

    const playbackItem = results[Math.min(this.ikState.playbackIndex, results.length - 1)];
    this.#applyRobotJointPose(this.ikState.importedJointRowsDeg[playbackItem.index]);
    this.viewer.setLabMarkers?.({
      actual: this.kinematics.getEndEffectorPositionMeters(),
    });
    this.viewer.setLabTrajectories?.({
      target: posesToViewerPoints(this.ikState.targetPath),
      actual: posesToViewerPoints(results.map((item) => item.actual.pose)),
    });
  }

  #startIkPlayback() {
    if (!this.ikState.lastResults.length) {
      this.setStatus("Please import an IK result CSV first.", "warn");
      return;
    }

    this.#stopIkPlayback();
    this.ikState.playbackTimer = window.setInterval(() => {
      this.#stepIkPlayback();
    }, this.ikState.speedMs);
  }

  #stopIkPlayback() {
    if (this.ikState.playbackTimer) {
      window.clearInterval(this.ikState.playbackTimer);
      this.ikState.playbackTimer = null;
    }
  }

  #stepIkPlayback() {
    if (!this.ikState.lastResults.length) return;

    if (this.ikState.mode === "single") {
      const active = this.ikState.lastResults[this.ikState.solutionIndex] || this.ikState.lastResults[0];
      this.#applyRobotJointPose(active.jointDeg);
      this.viewer.setLabMarkers?.({
        actual: this.kinematics.getEndEffectorPositionMeters(),
      });
      return;
    }

    this.ikState.playbackIndex =
      (this.ikState.playbackIndex + 1) % this.ikState.lastResults.length;
    const item = this.ikState.lastResults[this.ikState.playbackIndex];
    this.#applyRobotJointPose(this.ikState.importedJointRowsDeg[item.index]);
    this.viewer.setLabMarkers?.({
      actual: this.kinematics.getEndEffectorPositionMeters(),
    });
  }

  #refreshCurrentPaneVisualization() {
    if (this.currentPane === "dh") {
      this.#refreshDhInspect();
    } else if (this.currentPane === "fk") {
      this.#refreshFkComparison();
    } else {
      this.#refreshIkResults();
    }
  }

  #applyRobotJointPose(jointDeg) {
    if (!this.kinematics) return;
    this.kinematics.setJointVector(jointVectorDegToRad(jointDeg));
  }
}
