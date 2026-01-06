// --- 1. ডিফল্ট কনফিগারেশন ---
const defaultDH = [
    { a: 0, d: 201, alpha: 0, min: -360, max: 360 },
    { a: 0, d: 0, alpha: 90, min: -180, max: 180 },
    { a: 590, d: 0, alpha: 0, min: -160, max: 160 },
    { a: 560, d: 175, alpha: 0, min: -180, max: 180 },
    { a: 0, d: 140, alpha: -90, min: -180, max: 180 },
    { a: 0, d: 117, alpha: 90, min: -360, max: 360 }
];

window.onload = function() {
    const tbody = document.getElementById('dhBody');
    defaultDH.forEach((row, i) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `
            <td><strong>J${i+1}</strong></td>
            <td><input type="number" class="dh-a" value="${row.a}"></td>
            <td><input type="number" class="dh-d" value="${row.d}"></td>
            <td><input type="number" class="dh-alpha" value="${row.alpha}"></td>
            <td><input type="number" class="dh-min" value="${row.min}" style="color:#c0392b;"></td>
            <td><input type="number" class="dh-max" value="${row.max}" style="color:#27ae60;"></td>
        `;
        tbody.appendChild(tr);
    });
};

// --- MODAL & TAB FUNCTIONS ---
function openModal() { document.getElementById("helpModal").style.display = "block"; }
function closeModal() { document.getElementById("helpModal").style.display = "none"; }
// বাইরে ক্লিক করলে বন্ধ হবে
window.onclick = function(event) {
    if (event.target == document.getElementById("helpModal")) {
        closeModal();
    }
}
function openTab(evt, tabName) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) { tabcontent[i].style.display = "none"; }
    tablinks = document.getElementsByClassName("tablinks");
    for (i = 0; i < tablinks.length; i++) { tablinks[i].className = tablinks[i].className.replace(" active", ""); }
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
}


// --- MATH & KINEMATICS LOGIC ---
const Mat = {
    add: (A, B) => A.map((row, i) => row.map((val, j) => val + B[i][j])),
    sub: (A, B) => A.map((row, i) => row.map((val, j) => val - B[i][j])),
    dot: (A, B) => { 
        let result = Array(A.length).fill(0).map(() => Array(B[0].length).fill(0));
        for (let i = 0; i < A.length; i++) {
            for (let j = 0; j < B[0].length; j++) {
                for (let k = 0; k < A[0].length; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    },
    transpose: (A) => A[0].map((_, c) => A.map(r => r[c])),
    identity: (n) => Array(n).fill(0).map((_, i) => Array(n).fill(0).map((_, j) => i === j ? 1 : 0)),
    scale: (A, s) => A.map(row => row.map(val => val * s)),
    inv: (A) => {
        let n = A.length;
        let M = A.map((row, i) => [...row, ...Array(n).fill(0).map((_, j) => i === j ? 1 : 0)]);
        for (let i = 0; i < n; i++) {
            let pivot = M[i][i];
            if (Math.abs(pivot) < 1e-10) return null; 
            for (let j = 0; j < 2 * n; j++) M[i][j] /= pivot;
            for (let k = 0; k < n; k++) {
                if (k !== i) {
                    let factor = M[k][i];
                    for (let j = 0; j < 2 * n; j++) M[k][j] -= factor * M[i][j];
                }
            }
        }
        return M.map(row => row.slice(n));
    }
};

function toRad(deg) { return deg * Math.PI / 180; }
function toDeg(rad) { return rad * 180 / Math.PI; }

function getMatA(theta, d, a, alpha, type) {
    let ct = Math.cos(theta), st = Math.sin(theta);
    let ca = Math.cos(alpha), sa = Math.sin(alpha);
    if (type === 'standard') {
        return [[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0, 0, 0, 1]];
    } else {
        return [[ct, -st, 0, a], [st*ca, ct*ca, -sa, -d*sa], [st*sa, ct*sa, ca, d*ca], [0, 0, 0, 1]];
    }
}

function forwardKinematics(joints, dhParams, type) {
    let T = Mat.identity(4);
    for (let i = 0; i < 6; i++) {
        let theta = joints[i]; 
        let A = getMatA(theta, dhParams[i].d, dhParams[i].a, dhParams[i].alpha, type);
        T = Mat.dot(T, A);
    }
    return T;
}

function getTargetMatrix(x, y, z, rx, ry, rz) {
    let Rx = [[1,0,0,0], [0, Math.cos(rx), -Math.sin(rx), 0], [0, Math.sin(rx), Math.cos(rx), 0], [0,0,0,1]];
    let Ry = [[Math.cos(ry), 0, Math.sin(ry), 0], [0,1,0,0], [-Math.sin(ry), 0, Math.cos(ry), 0], [0,0,0,1]];
    let Rz = [[Math.cos(rz), -Math.sin(rz), 0, 0], [Math.sin(rz), Math.cos(rz), 0, 0], [0,0,1,0], [0,0,0,1]];
    let Trans = [[1,0,0,x], [0,1,0,y], [0,0,1,z], [0,0,0,1]];

    // --- CHANGE IS HERE ---
    // RoboDK (Generic) uses Rx * Ry * Rz order
    let Rotation = Mat.dot(Rx, Mat.dot(Ry, Rz)); 
    
    return Mat.dot(Trans, Rotation);
    //return Mat.dot(Trans, Mat.dot(Rz, Mat.dot(Ry, Rx)));
}

function calculateError(T_target, T_curr) {
    let pos_err = [T_target[0][3]-T_curr[0][3], T_target[1][3]-T_curr[1][3], T_target[2][3]-T_curr[2][3]];
    let n_cur = [T_curr[0][0], T_curr[1][0], T_curr[2][0]], o_cur = [T_curr[0][1], T_curr[1][1], T_curr[2][1]], a_cur = [T_curr[0][2], T_curr[1][2], T_curr[2][2]];
    let n_tar = [T_target[0][0], T_target[1][0], T_target[2][0]], o_tar = [T_target[0][1], T_target[1][1], T_target[2][1]], a_tar = [T_target[0][2], T_target[1][2], T_target[2][2]];
    
    function cross(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
    let e1 = cross(n_cur, n_tar), e2 = cross(o_cur, o_tar), e3 = cross(a_cur, a_tar);
    let rot_err = [0.5*(e1[0]+e2[0]+e3[0]), 0.5*(e1[1]+e2[1]+e3[1]), 0.5*(e1[2]+e2[2]+e3[2])];
    return [...pos_err, ...rot_err];
}

function calculateJacobian(joints, dhParams, type) {
    let J = Array(6).fill(0).map(() => Array(6).fill(0));
    let delta = 0.0001;
    let T_curr = forwardKinematics(joints, dhParams, type);
    for (let i = 0; i < 6; i++) {
        let perturbed = [...joints];
        perturbed[i] += delta;
        let T_new = forwardKinematics(perturbed, dhParams, type);
        let err = calculateError(T_new, T_curr);
        for (let r = 0; r < 6; r++) J[r][i] = err[r] / delta;
    }
    return J;
}

// --- 4. Main Solver (Updated with Random Restart) ---
function calculateIK() {
    let x = parseFloat(document.getElementById('tx').value);
    let y = parseFloat(document.getElementById('ty').value);
    let z = parseFloat(document.getElementById('tz').value);
    let rx = toRad(parseFloat(document.getElementById('rx').value));
    let ry = toRad(parseFloat(document.getElementById('ry').value));
    let rz = toRad(parseFloat(document.getElementById('rz').value));
    let dhType = document.getElementById('dhType').value;

    let dhParams = [];
    let rows = document.getElementById('dhBody').rows;
    for (let row of rows) {
        dhParams.push({
            a: parseFloat(row.querySelector('.dh-a').value),
            d: parseFloat(row.querySelector('.dh-d').value),
            alpha: toRad(parseFloat(row.querySelector('.dh-alpha').value)),
            min: toRad(parseFloat(row.querySelector('.dh-min').value)),
            max: toRad(parseFloat(row.querySelector('.dh-max').value))
        });
    }

    // 1. Get Target Matrix (Generic Order: Rx * Ry * Rz for RoboDK match)
    let T_target = getTargetMatrix(x, y, z, rx, ry, rz);

    // --- SMART SOLVER LOGIC START ---
    
    // আমরা ৫ বার চেষ্টা করব ভিন্ন ভিন্ন পজিশন থেকে
    let attempt = 0;
    let max_attempts = 5; 
    let solved = false;
    let final_q = [];
    
    // Initial Guesses (প্রথমবার তোমার দেওয়াটা, পরেরগুলো র‍্যান্ডম)
    let guesses = [
        [0, toRad(-30), toRad(90), 0, 0, 0], // Guess 1: Default
        [0, 0, 0, 0, 0, 0],                  // Guess 2: Zero
        [toRad(90), toRad(-45), toRad(45), 0, 0, 0] // Guess 3: Another pose
    ];

    // মেইন লুপ: যতক্ষণ সলভ না হয় অথবা ৫ বার চেষ্টা শেষ না হয়
    while (!solved && attempt < max_attempts) {
        
        // যদি লিস্টের গেস শেষ হয়ে যায়, র‍্যান্ডম ভ্যালু নাও
        let q;
        if (attempt < guesses.length) {
            q = [...guesses[attempt]];
        } else {
            // Random guess within limits (Simple randomization)
            q = dhParams.map(p => (Math.random() * (p.max - p.min) + p.min) * 0.5); 
        }

        let max_iter = 600; // ইটারেশন একটু বাড়ালাম
        let lambda = 0.01;  // Damping factor

        for (let k = 0; k < max_iter; k++) {
            let T_curr = forwardKinematics(q, dhParams, dhType);
            let error = calculateError(T_target, T_curr);
            
            // এরর খুব কম হলে লুপ ব্রেক (Success)
            let errorNorm = Math.sqrt(error.reduce((a, b) => a + b * b, 0));
            if (errorNorm < 1e-3) { 
                solved = true;
                break;
            }

            let J = calculateJacobian(q, dhParams, dhType);
            let JT = Mat.transpose(J);
            let JJT = Mat.dot(J, JT);
            let damping = Mat.scale(Mat.identity(6), lambda * lambda);
            
            let A = Mat.add(JJT, damping);
            let A_inv = Mat.inv(A);

            if (!A_inv) break; // Singularity হলে এই চেষ্টা বাদ

            let dq = Mat.dot(Mat.dot(JT, A_inv), error.map(e => [e]));

            for (let i = 0; i < 6; i++) {
                q[i] += dq[i][0];
                // Limits check
                if (q[i] > dhParams[i].max) q[i] = dhParams[i].max;
                if (q[i] < dhParams[i].min) q[i] = dhParams[i].min;
            }
        }

        if (solved) {
            final_q = q; // সলিউশন পাওয়া গেছে!
        }
        attempt++;
    }
    
    // যদি কোনোভাবেই সলভ না হয়, শেষ চেষ্টাটাই রেজাল্ট হিসেবে দেখাবে
    if (!solved) final_q = guesses[0]; 

    // --- OUTPUT RESULTS ---
    document.getElementById('result').style.display = 'block';
    let status = document.getElementById('statusText');
    
    if (solved) {
        status.innerHTML = `Converged in attempt #${attempt} ✅`;
        status.style.color = "green";
    } else {
        status.innerHTML = "Could not converge (Target unreachable?) ❌";
        status.style.color = "red";
    }

    // 1. Joint Angles Display
    let display = document.getElementById('jointsDisplay');
    display.innerHTML = "";
    final_q.forEach((val, i) => {
        display.innerHTML += `
            <div class="joint-box">
                <div class="joint-label">Joint ${i+1}</div>
                <div class="joint-val">${toDeg(val).toFixed(2)}°</div>
            </div>`;
    });

    // 2. Matrix Verification Display
    let T_achieved = forwardKinematics(final_q, dhParams, dhType);
    let T_diff = Mat.sub(T_target, T_achieved);

    renderMatrix(T_target, 'matTarget');
    renderMatrix(T_achieved, 'matAchieved');
    renderMatrix(T_diff, 'matDiff');
}

function renderMatrix(mat, elementId) {
    let html = '<table class="mat-table">';
    for(let i=0; i<4; i++) {
        html += '<tr>';
        for(let j=0; j<4; j++) {
            let val = mat[i][j];
            let color = (elementId === 'matDiff' && Math.abs(val) > 0.01) ? 'red' : 'black';
            html += `<td style="color:${color}">${val.toFixed(3)}</td>`;
        }
        html += '</tr>';
    }
    html += '</table>';
    document.getElementById(elementId).innerHTML = html;
}